sudo /opt/etherlab/bin/ethercat download --type=uint32 --position=0 0x1400 1 0xc0000201

sudo /opt/etherlab/bin/ethercat download --type=uint8 --position=0 0x1600 0 0

sudo /opt/etherlab/bin/ethercat download --type=uint32 --position=0 0x1600 1 0x60400010

sudo /opt/etherlab/bin/ethercat download --type=uint32 --position=0 0x1600 2 0x60810020

sudo /opt/etherlab/bin/ethercat download --type=uint8 --position=0 0x1600 0 2

sudo /opt/etherlab/bin/ethercat download --type=uint8 --position=0 0x1400 2 0xfe

sudo /opt/etherlab/bin/ethercat download --type=uint32 --position=0 0x1400 1 0x40000201

sudo /opt/etherlab/bin/ethercat download --type=uint32 --position=0 0x1401 1 0xc0000301

sudo /opt/etherlab/bin/ethercat download --type=uint8 --position=0 0x1601 0 0

sudo /opt/etherlab/bin/ethercat download --type=uint32 --position=0 0x1601 1 0x60ff0020

sudo /opt/etherlab/bin/ethercat download --type=uint32 --position=0 0x1601 2 0x60830020

sudo /opt/etherlab/bin/ethercat download --type=uint8 --position=0 0x1601 0 2

sudo /opt/etherlab/bin/ethercat download --type=uint8 --position=0 0x1401 2 0xfe

sudo /opt/etherlab/bin/ethercat download --type=uint32 --position=0 0x1401 1 0x40000301

#everything in 0.1 deg and secs
sudo /opt/etherlab/bin/ethercat download --position=0 --type uint32 0x6093 1 8192
sudo /opt/etherlab/bin/ethercat download --position=0 --type uint32 0x6093 2 25
sudo /opt/etherlab/bin/ethercat download --position=0 --type uint32 0x6094 1 3
sudo /opt/etherlab/bin/ethercat download --position=0 --type uint32 0x6094 2 10
sudo /opt/etherlab/bin/ethercat download --position=0 --type uint32 0x6097 1 384
sudo /opt/etherlab/bin/ethercat download --position=0 --type uint32 0x6097 2 5
sudo /opt/etherlab/bin/ethercat download --position=0 --type uint32 0x2090 2 144000
sudo /opt/etherlab/bin/ethercat download --position=0 --type uint32 0x2090 3 144000
sudo /opt/etherlab/bin/ethercat download --position=0 --type uint32 0x2090 4 144000
sudo /opt/etherlab/bin/ethercat download --position=0 --type uint32 0x2090 5 144000

