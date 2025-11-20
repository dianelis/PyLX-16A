#!/bin/bash
# Dance script wrapper
# Usage: sh dance.sh or ./dance.sh

cd "$(dirname "$0")"
./venv/bin/python3 dance.py

