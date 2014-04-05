/opt/etherlab/bin/ethercat download --type=uint32 --position=$1 0x1401 1 0xc0000301

/opt/etherlab/bin/ethercat download --type=uint8 --position=$1 0x1601 0 0

/opt/etherlab/bin/ethercat download --type=uint32 --position=$1 0x1601 1 0x60ff0020

/opt/etherlab/bin/ethercat download --type=uint32 --position=$1 0x1601 2 0x60830020

/opt/etherlab/bin/ethercat download --type=uint8 --position=$1 0x1601 0 2

/opt/etherlab/bin/ethercat download --type=uint8 --position=$1 0x1401 2 0xfe

/opt/etherlab/bin/ethercat download --type=uint32 --position=$1 0x1401 1 0x40000301
