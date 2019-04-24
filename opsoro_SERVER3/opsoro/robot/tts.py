import os
import subprocess


def create_espeak(self, text, file_path, language, gender, delay, speed):
    """
    Convert text to speech using the espeak TTS library.

    :param string text:         text to convert to speech
    :param string file_path:    file path to store the speech soundfile
    :param string language:     language initials
    :param string gender:       specify gender (m for male, f for female)
    :param int delay:           delay between words in ms
    :param int speed:           speed in words-per-minute
    """
    text = "\"" + text + "\""
    subprocess.call(["espeak", "-v", language + "+" + gender + "3", "-g", delay, "-s", speed, "-w", file_path, text])
