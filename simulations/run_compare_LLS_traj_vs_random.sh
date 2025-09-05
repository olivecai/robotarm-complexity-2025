#!/bin/bash
echo > BASHAUTOMATION_LLS_jacobian_error_compare_for_plot1.txt
for robot in dof2 dof3 jaco kinova
do
echo "robot: ${robot}" >> BASHAUTOMATION_LLS_jacobian_error_compare_for_plot1.txt
echo "robot: ${robot}"
    for i in {1..10}; do

    echo "generating batch" $i  >> BASHAUTOMATION_LLS_jacobian_error_compare_for_plot1.txt
    echo "generating batch" $i
    python3 LLS_traj_vs_random.py ${robot} > temp.txt
    tail -n 4 temp.txt >> BASHAUTOMATION_LLS_jacobian_error_compare_for_plot1.txt
    done
done


