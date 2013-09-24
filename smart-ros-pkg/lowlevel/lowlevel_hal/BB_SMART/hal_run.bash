#!/bin/bash
echo "[CONFIG]">hal_config.ini
package_path=`rospack find lowlevel_hal`
echo "PATH="$package_path>>hal_config.ini
halrun -f $package_path/BB_SMART/BeBoGolfcartROSAdapter.hal -i hal_config.ini -I
