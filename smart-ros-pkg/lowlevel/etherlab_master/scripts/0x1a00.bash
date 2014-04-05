/opt/etherlab/bin/ethercat download --type=uint32 --position=$1 0x1800 1 0xc0000181

/opt/etherlab/bin/ethercat download --type=uint8 --position=$1 0x1a00 0 0

/opt/etherlab/bin/ethercat download --type=uint32 --position=$1 0x1a00 1 0x60610008

/opt/etherlab/bin/ethercat download --type=uint32 --position=$1 0x1a00 2 0x60410010

/opt/etherlab/bin/ethercat download --type=uint32 --position=$1 0x1a00 3 0x606c0020

/opt/etherlab/bin/ethercat download --type=uint32 --position=$1 0x1a00 4 0x10010008

/opt/etherlab/bin/ethercat download --type=uint8 --position=$1 0x1a00 0 4

/opt/etherlab/bin/ethercat download --type=uint8 --position=$1 0x1800 2 0xfe

/opt/etherlab/bin/ethercat download --type=uint32 --position=$1 0x1800 1 0x40000181
