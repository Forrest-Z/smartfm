#!/bin/bash
# full system backup

# Backup destination
backdest=/media/rosbag

# Labels for backup name
date=$(date "+%F")
backupfile="$backdest/full_os_backup_$date.tar.gz"

# Files to exclude
exclude_file=`mktemp --tmpdir=/tmp full_os_backup_exclude_list.XXXXXX`

cat > $exclude_file <<EOF
tmp
*.bag
*.mpg
*.mpeg
*.avi
EOF


sudo tar czpvf $backupfile --one-file-system --exclude-from=$exclude_file --directory=/ .

exit