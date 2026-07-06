# Install OpenVLA-OFT dependencies
pip install peft==0.11.1 accelerate==1.10.1 sentencepiece==0.1.99 timm==0.9.10 pyyaml --break-system-packages
pip install  git+https://github.com/moojink/transformers-openvla-oft.git --break-system-packages
pip install tensorflow json_numpy einops --break-system-packages
pip install packaging ninja --break-system-packages
# Verify Ninja --> should return exit code "0"
# check if ninja if return exit code 0, if not, install ninja
ninja --version; echo $0
apt-get install cuda-toolkit-12-8 -y
update-alternatives --set cuda /usr/local/cuda-12.8
pip install "flash-attn==2.5.5" --no-build-isolation --break-system-packages
pip install --no-deps "dlimp @ git+https://github.com/moojink/dlimp_openvla" --break-system-packages
pip install -U protobuf --break-system-packages

