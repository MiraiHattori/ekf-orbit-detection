#!/usr/bin/env bash

cat log | grep pixel > pixel.log
cat log | grep pos > pos.log
cat log | grep measure > measure.log
cat log | grep estimated > estimated.log

gnuplot -e "
    set view equal xyz;
    splot \"measure.log\" using 2:3:4 with linespoint title \"measured point\",
          \"pos.log\" using 3:4:5 title \"real point\",
          \"estimated.log\" using 2:3:4 title \"estimated point\",
          \"estimated.log\" using 2:3:4:((\$5)*0.1):((\$6)*0.1):((\$7)*0.1):(sqrt((\$5)*(\$5)+(\$6)*(\$6)+(\$7)*(\$7))*0.01) w vector lc palette title \"array\";
    pause -1;
"

#          \"estimated.log\" using 2:3:4:((\$5)*0.01):((\$6)*0.01):((\$7)*0.01):(sqrt((\$5)*(\$5)+(\$6)*(\$6)+(\$7)*(\$7))*10.0) w vector lc palette title \"array\";
   # pause mouse, key;
# plot \"pixel.log\" using 2:3, \"pixel.log\" using 4:5;
