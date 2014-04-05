/opt/etherlab/bin/ethercat download --type=uint32 --position=$1 0x1801 1 0xc0000281

/opt/etherlab/bin/ethercat download --type=uint8 --position=$1 0x1a01 0 0

/opt/etherlab/bin/ethercat download --type=uint32 --position=$1 0x1a01 1 0x60640020

/opt/etherlab/bin/ethercat download --type=uint32 --position=$1 0x1a01 2 0x60780010

/opt/etherlab/bin/ethercat download --type=uint32 --position=$1 0x1a01 3 0x60770010

/opt/etherlab/bin/ethercat download --type=uint8 --position=$1 0x1a01 0 3

/opt/etherlab/bin/ethercat download --type=uint8 --position=$1 0x1801 2 0xfe

/opt/etherlab/bin/ethercat download --type=uint32 --position=$1 0x1801 1 0x40000281
