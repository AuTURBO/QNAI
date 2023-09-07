#! /bin/bash

# Download assets from google drive
tutorial_asset=https://www.dropbox.com/scl/fi/5k9ik29pdgnevhvpf17f5/tutorial_asset.zip?rlkey=ysip5tuzrml2ys7dlqqqr6c4l

wget -O tutorial_asset.zip "$tutorial_asset"

# unzip files and remove .zip files
unzip tutorial_asset.zip -d ./ && rm tutorial_asset.zip