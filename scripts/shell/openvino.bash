#!bash
echo "start install curl"
sudo apt install curl
echo "successfully install curl"
curl https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB > /tmp/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
if [-f /tmp/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB ]; then
    echo "successfully wget  GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB"
else
    echo "wget GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB failed"
    exit -1
fi

sudo apt-key add /tmp/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB 

echo "deb https://apt.repos.intel.com/openvino/2023 ubuntu22 main" | sudo tee /etc/apt/sources.list.d/intel-openvino-2023.list # ubuntu22

sudo apt update

apt-cache search openvino

sudo apt install openvino
echo "successfully install openvino"
