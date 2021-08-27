#!/usr/bin/env python3

from datetime import datetime

class BaseLogger():
    def __init__(self):
        pass
    def log(self, debug_message):
        print(f'{datetime.now()}: {debug_message}')

logger = BaseLogger()

def log(debug_message):
    logger.log(debug_message)

def test():
    log("testing 123")

if __name__ == "__main__":
    test()
