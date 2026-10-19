export PATH="$PATH:$HOME/.local/bin"
export MAMBA_ROOT_PREFIX="$HOME/.micromamba"
bindkey '^y' autosuggest-accept
alias fix='cp ~/.zsh_history ~/.zsh_history_$(date +"%Y%m%d%H%M%S") && strings ~/.zsh_history_$(date +"%Y%m%d%H%M%S") > ~/.zsh_history'
alias cm='catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo -j6'
alias rk='rosnode kill -a && sudo pkill -KILL rosmaster && sleep 0.5 && sudo pkill -KILL rosmaster'
alias run='cd ~/dynamic_grasp && ./scripts/run.sh'
alias sb='cd ~/dynamic_grasp && ./scripts/run.sh state'
alias vb='cd ~/dynamic_grasp && ./scripts/run.sh relay'
alias dg='cd ~/dynamic_grasp && ./scripts/run.sh dg'
alias rel='cd ~/dynamic_grasp && ./scripts/pub_release.sh'
alias bag='cd ~/dynamic_grasp/bags && ./bag.sh'
export LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1 # for VideoCapture
