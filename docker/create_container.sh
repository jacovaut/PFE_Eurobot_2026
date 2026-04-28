#!/bin/bash

docker builder prune -a -f
docker system prune -a -f

docker build -t "$IMAGE_NAME" .
