#!/bin/bash
INIT_CMD="$(cat download.jlink)"
(echo "${INIT_CMD}" && cat) | JLinkExe
