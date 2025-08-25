#!/bin/bash
URL="http://192.168.186.2"  # Replace with the URL you want to test
if wget --quiet --spider "$URL"; then
  echo "Success: $URL is reachable"
else
  echo "Failed: $URL is not reachable"
fi