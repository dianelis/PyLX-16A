#!/bin/bash
# Walk script wrapper
# Usage: sh walk.sh or ./walk.sh

cd "$(dirname "$0")"
./venv/bin/python3 walk.py

