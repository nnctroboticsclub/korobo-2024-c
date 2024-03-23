tms:
	tmux start

ct: tms
	tmux set-option -g default-terminal screen-256color
	tmux set -g terminal-overrides 'xterm:colors=256'

	tmux set-option -g status-position top
	tmux set-option -g status-left-length 90
	tmux set-option -g status-right-length 90

	tmux set-option -g status-left "#S/#I:#W.#P"
	tmux set-option -g status-right ''
	tmux set-option -g status-justify centre
	tmux set-option -g status-bg "colour238"
	tmux set-option -g status-fg "colour255"

	tmux bind '|' split-window -h
	tmux bind '-' split-window -v

	tmux set-option -g mouse on
	tmux bind -n WheelUpPane if-shell -F -t = "#{mouse_any_flag}" "send-keys -M" "if -Ft= '#{pane_in_mode}' 'send-keys -M' 'copy-mode -e'"

it:
	tmux new-session -d -s korobo2023 -n work

	tmux new-window -t korobo2023:1 -n log
	tmux split-window -t korobo2023:log.0 -v
	tmux split-window -t korobo2023:log.0 -h
	tmux split-window -t korobo2023:log.2 -h
	tmux split-window -t korobo2023:log.3 -h
	tmux resize-pane -t korobo2023:log.1 -x 20

	tmux select-window -t korobo2023:work
	tmux select-window -t korobo2023:log
	tmux send-keys -t korobo2023:log.0 'make me' C-m
	tmux send-keys -t korobo2023:log.1 'make ws' C-m
	tmux send-keys -t korobo2023:log.2 'make ms1' C-m
	tmux send-keys -t korobo2023:log.3 'make ms2' C-m
	tmux send-keys -t korobo2023:log.4 'cd ws-relay; python3 main.py' C-m

at:
	tmux attach -t korobo2023

stm: ct it at
stw:
	tmux new-session -d -s korobo2023-work -n work
	tmux attach -t korobo2023-work