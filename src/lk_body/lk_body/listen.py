"""Microphone. Savelij does not record.

The USB device has a speaker port and an input. This body uses the speaker
and leaves every input closed.
"""


class HoldMic:
    def start(self) -> bool:
        return False

    def stop(self) -> bytes:
        return b""
