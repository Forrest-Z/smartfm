/opt/etherlab/bin/ethercat download --type=uint32 --position=$1 0x1400 1 0xc0000201

/opt/etherlab/bin/ethercat download --type=uint8 --position=$1 0x1600 0 0

/opt/etherlab/bin/ethercat download --type=uint32 --position=$1 0x1600 1 0x60400010

/opt/etherlab/bin/ethercat download --type=uint32 --position=$1 0x1600 2 0x60810020

/opt/etherlab/bin/ethercat download --type=uint8 --position=$1 0x1600 0 2

/opt/etherlab/bin/ethercat download --type=uint8 --position=$1 0x1400 2 0xfe

/opt/etherlab/bin/ethercat download --type=uint32 --position=$1 0x1400 1 0x40000201
