#!/usr/bin/env python3
"""Offline speech recognition node.

Captures microphone audio with arecord, recognizes speech with Vosk,
and publishes the recognized text on /voice_commands (std_msgs/String).
"""

import audioop
import json
import os
import subprocess

import rospy
from std_msgs.msg import String

from vosk import Model, KaldiRecognizer, SetLogLevel

SAMPLE_RATE = 16000
CHUNK_SIZE = 4000  # bytes read from arecord per iteration

# Restricting the grammar to the phrases we care about makes
# recognition much more reliable than free-form dictation.
GRAMMAR = [
    "move forward", "go forward", "forward", "go",
    "move backward", "go back", "backward", "back", "reverse", "turn around",
    "turn left", "left",
    "turn right", "right",
    "stop", "halt",
    "faster", "go faster", "move faster", "speed up",
    "slower", "go slower", "move slower", "slow down",
    "[unk]",
]


def unmute_microphone():
    """Best-effort unmute: ALSA capture gets re-muted on some systems
    (e.g. restored by PulseAudio), which silently breaks recognition."""
    for cmd in (["amixer", "sset", "Capture", "100%", "cap"],
                ["pactl", "set-source-mute", "@DEFAULT_SOURCE@", "0"],
                ["pactl", "set-source-volume", "@DEFAULT_SOURCE@", "100%"]):
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL, timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            pass


def main():
    rospy.init_node("voice_recognizer")
    unmute_microphone()

    default_model = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "models", "vosk-model-small-en-us-0.15")
    model_path = rospy.get_param("~model_path", default_model)
    device = rospy.get_param("~device", "default")  # ALSA capture device
    use_grammar = rospy.get_param("~use_grammar", True)
    # Software amplification for weak microphones (1.0 = no gain).
    gain = rospy.get_param("~gain", 1.0)

    # Other packages can supply their own vocabulary (e.g. room names) via
    # ~grammar; leaving it unset keeps the teleop phrases above.
    grammar = rospy.get_param("~grammar", None)
    if grammar:
        if isinstance(grammar, str):
            grammar = [p.strip() for p in grammar.split(",") if p.strip()]
        if "[unk]" not in grammar:
            grammar = list(grammar) + ["[unk]"]
    else:
        grammar = GRAMMAR

    if not os.path.isdir(model_path):
        rospy.logfatal("Vosk model not found at %s", model_path)
        return

    SetLogLevel(-1)
    rospy.loginfo("Loading Vosk model from %s ...", model_path)
    model = Model(model_path)
    if use_grammar:
        rospy.loginfo("Grammar: %d phrases", len(grammar))
        recognizer = KaldiRecognizer(model, SAMPLE_RATE, json.dumps(grammar))
    else:
        recognizer = KaldiRecognizer(model, SAMPLE_RATE)

    pub = rospy.Publisher("voice_commands", String, queue_size=10)

    arecord_cmd = [
        "arecord", "-D", device, "-q",
        "-f", "S16_LE", "-r", str(SAMPLE_RATE), "-c", "1", "-t", "raw",
    ]
    rospy.loginfo("Starting microphone capture: %s", " ".join(arecord_cmd))
    proc = subprocess.Popen(arecord_cmd, stdout=subprocess.PIPE)

    # The hint has to follow the vocabulary actually loaded - suggesting the
    # teleop phrases while a caller-supplied grammar is active just misleads.
    if grammar is GRAMMAR:
        rospy.loginfo("Listening... say e.g. 'move forward', 'turn left', 'stop'")
    else:
        rospy.loginfo("Listening... vocabulary: %s", ", ".join(
            w for w in grammar if w != "[unk]"))
    try:
        while not rospy.is_shutdown():
            data = proc.stdout.read(CHUNK_SIZE)
            if not data:
                rospy.logerr("Microphone stream ended unexpectedly")
                break
            if gain != 1.0:
                data = audioop.mul(data, 2, gain)
            if recognizer.AcceptWaveform(data):
                text = json.loads(recognizer.Result()).get("text", "")
                text = text.replace("[unk]", "").strip()
                if text:
                    rospy.loginfo("Heard: '%s'", text)
                    pub.publish(text)
    finally:
        proc.terminate()
        proc.wait()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
