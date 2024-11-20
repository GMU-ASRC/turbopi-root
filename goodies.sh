#!/bin/bash

if [ "$(id -u)" -eq 0 ]; then
        echo 'This script should not be run by root' >&2
        exit 1
fi


BASHRC=$(realpath ~/.bashrc)

function update_bashrc {
    LINE=$1  # arg 1
    eval $LINE  # add to current environment
    grep -qF -- "$LINE" "$BASHRC"
    RES=$?
    if [ $RES -eq 1 ]; then
        echo "$LINE" >> "$BASHRC"
        return 0
    else
        return 1
    fi
}

function removefrom_bashrc {
    LINE=$1
    grep -v "$LINE" $BASHRC > temp && mv temp $BASHRC
    return
}


sudo apt update


# if pyenv is not installed, install it
if ! command -v pyenv &> /dev/null
then
	sudo apt install -y build-essential libssl-dev zlib1g-dev \
	libbz2-dev libreadline-dev libsqlite3-dev curl git \
	libncursesw5-dev xz-utils tk-dev libxml2-dev libxmlsec1-dev libffi-dev liblzma-dev

	curl https://pyenv.run | bash

	echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.bashrc
	echo 'command -v pyenv >/dev/null || export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.bashrc
	echo 'eval "$(pyenv init -)"' >> ~/.bashrc

	source ~/.bashrc
fi

sudo git clone --depth=1 https://github.com/amix/vimrc.git /opt/vim_runtime
# to install for all users with home directories, note that root will not be included
sudo bash /opt/vim_runtime/install_awesome_parameterized.sh /opt/vim_runtime --all
sudo bash /opt/vim_runtime/install_awesome_parameterized.sh /opt/vim_runtime root

# if zoxide is not installed, install it
if ! command -v zoxide &> /dev/null
then
	echo "Installing zoxide"
	curl -sSfL https://raw.githubusercontent.com/ajeetdsouza/zoxide/main/install.sh | sh
	update_bashrc "export PATH=\"\$HOME/.local/bin:\$PATH\""
	echo "updating bashrc with zoxide init"
	update_bashrc "eval \"\$(zoxide init bash --cmd cd)\""
else
	echo "updating bashrc with zoxide init"
	update_bashrc "eval \"\$(zoxide init bash --cmd cd)\""
fi

# if fzf is not installed, install it
if ! command -v fzf &> /dev/null
then
	echo "Installing fzf"
	git clone --depth 1 https://github.com/junegunn/fzf.git $HOME/.fzf
	$HOME/.fzf/install --all
fi

# remove old aliases
removefrom_bashrc "eval \"\$(zoxide init bash)\""
removefrom_bashrc "alias cd=z"

sudo apt install ncdu bat aptitude vim -y

update_bashrc "alias bat='batcat'"

sudo apt install exa -y


if command -v exa &> /dev/null
then
	update_bashrc "alias e='exa'"
	update_bashrc "alias l='exa -F'"
	update_bashrc "alias la='exa -a'"
	update_bashrc "alias ll='exa -laF'"
	update_bashrc "alias lls='exa -la --sort=size'"
	update_bashrc "alias llt='exa -la --sort=time'"
else
	# standard shell stuff for if exa did not install properly
	update_bashrc "alias ls='ls --color --classify --human-readable'"
	update_bashrc "alias l='ls -CF'"
	update_bashrc "alias la='ls -A'"
	update_bashrc "alias ll='ls -laF'"
	update_bashrc "alias lls='ls -la --sort=size'"
	update_bashrc "alias llt='ls -la --sort=time'"
fi

# great! now a bunch of stuff is installed by default.
# other stuff you may wish to do:
# Install rust:
# curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh