#!/bin/bash
echo >     BASHAUTOMATION_VS9.txt #clear the file
for robot in dof2 dof3
do
echo "robot: ${robot}" >>     BASHAUTOMATION_VS9.txt
echo "robot: ${robot}"
for i in {1..70}
do
    echo "generating batch" $i
    python3 LLS_traj_vs_random.py ${robot} > temp.txt
    tail -n 1 temp.txt >>     BASHAUTOMATION_VS9.txt
    done
done


