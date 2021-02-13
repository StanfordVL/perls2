#!/bin/sh

# Kill all three terminals for nuc_control since kill-session does not have sudo access.

# Set up tmux
session=$1

unset -f _tmux_send_keys_all_panes_

_tmux_send_keys_all_panes_() {
  for _pane in $(tmux list-panes -s -t "$session" -F '#P'); do
    tmux send-keys -t ${_pane} C-c C-m
  done
}




_tmux_send_keys_all_panes_ 

tmux kill-session -t $session
