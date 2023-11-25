#!bash
echo "start install curl"
sudo apt update && sudo apt install curl -y
echo "successfully install curl"
curl https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB > /tmp/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
if [ -f /tmp/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB ]; then
    echo "successfully curl GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB"
else
    echo "curl GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB failed"
    exit -1
fi

sudo apt-key add /tmp/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB 

echo "deb https://apt.repos.intel.com/openvino/2023 ubuntu22 main" | sudo tee /etc/apt/sources.list.d/intel-openvino-2023.list # ubuntu22

sudo apt update && sudo apt install openvino -y
echo "successfully install openvino"
