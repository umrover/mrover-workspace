# Bash shell tab completions for Jarvis

_complete_jarvis() {
    local candidates

    candidates=$(jarvis --complete -- ${COMP_WORDS[*]})

    COMPREPLY=( $(compgen -W "${candidates}" -- ${2}) )
}

complete -F _complete_jarvis -o default jarvis
