sudo /opt/etherlab/bin/ethercat download --type=uint32 --position=$1 0x1800 1 0xc0000181

sudo /opt/etherlab/bin/ethercat download --type=uint8 --position=$1 0x1a00 0 0

sudo /opt/etherlab/bin/ethercat download --type=uint32 --position=$1 0x1a00 1 0x60610008

sudo /opt/etherlab/bin/ethercat download --type=uint32 --position=$1 0x1a00 2 0x60410010

sudo /opt/etherlab/bin/ethercat download --type=uint32 --position=$1 0x1a00 3 0x606c0020

sudo /opt/etherlab/bin/ethercat download --type=uint32 --position=$1 0x1a00 4 0x10010008

sudo /opt/etherlab/bin/ethercat download --type=uint8 --position=$1 0x1a00 0 4

sudo /opt/etherlab/bin/ethercat download --type=uint8 --position=$1 0x1800 2 0xfe

sudo /opt/etherlab/bin/ethercat download --type=uint32 --position=$1 0x1800 1 0x40000181
