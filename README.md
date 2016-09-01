# QLogic BR-1020 Converged Network Adapter Driver
-----

### Installation
1. Clone the repository
	`git clone https://github.com/vikingix/bna.git`
2. Build the kernel module
	`make`
3. Install the kernel module
	`sudo make install`
4. Disable Generic  Segmentation Offload
	Add `options bna bnad_gro_disable=1` to /etc/modprobe.d/bna.conf
5. Update initramfs
	`sudo update-initramfs -u`
6. Insert the module manually (optional)
	`sudo modprobe bna`
