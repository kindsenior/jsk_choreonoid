#!/usr/bin/env python

from logging import getLogger, StreamHandler, DEBUG, INFO, WARNING, ERROR, CRITICAL
logger = getLogger(__name__)
handler = StreamHandler()
# logger.setLevel(DEBUG)
# logger.setLevel(INFO)
# logger.setLevel(ERROR)
logger.setLevel(CRITICAL)
logger.addHandler(handler)
logger.propagate = False
