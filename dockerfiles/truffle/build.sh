#!/bin/bash

# Assemble docker image. 
echo 'Building truffle docker image for smart contracts deployment.'

docker build -t truffle .