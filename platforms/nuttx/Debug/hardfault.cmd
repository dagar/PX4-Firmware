break up_hardfault
commands
printf "PX4 hardfault handler: setting PC to LR\n"
set $pc = $lr
printf "PX4 hardfault handler: instruction stepping 19 times\n"
stepi 19
end
