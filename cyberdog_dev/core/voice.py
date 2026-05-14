"""Minimal voice controller for required Stage 4 announcements."""


class VoiceController:
    def __init__(self, logger=None):
        self.logger = logger
        self.history: list[str] = []

    def say(self, text: str) -> None:
        self.history.append(text)
        if self.logger is not None:
            self.logger.info(f"语音播报: {text}")
        else:
            print(f"语音播报: {text}")
