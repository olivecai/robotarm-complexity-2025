#!/bin/bash
echo >  BASHAUTOMATION_VS.txt #clear the file
for robot in dof2 dof3 jaco kinova
do
echo "robot: ${robot}" >>  BASHAUTOMATION_VS.txt
echo "robot: ${robot}"
for i in {1..5}
do
    echo "generating batch" $i
    python3 LLS_traj_vs_random.py ${robot} > temp.txt
    tail -n 1 temp.txt >>  BASHAUTOMATION_VS.txt
    done
done


