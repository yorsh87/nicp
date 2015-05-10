#!/bin/bash

python ../../scripts/evaluate_rpe.py groundtruth.txt gicp_odometry.txt --verbose > gicp_result.txt
python ../../scripts/evaluate_rpe.py groundtruth.txt nicp_odometry_33.txt --verbose > nicp_result_33.txt
python ../../scripts/evaluate_rpe.py groundtruth.txt nicp_odometry_66.txt --verbose > nicp_result_66.txt
python ../../scripts/evaluate_rpe.py groundtruth.txt nicp_odometry.txt --verbose > nicp_result.txt

echo -n	"`grep "translational_error.mean" gicp_result.txt | awk '{ print $2 }'` " > morphing_result.txt
echo -n	"`grep "translational_error.std" gicp_result.txt | awk '{ print $2 }'` " >> morphing_result.txt
echo -n	"`grep "rotational_error.mean" gicp_result.txt | awk '{ print $2 }'` " >> morphing_result.txt
echo 	"`grep "rotational_error.std" gicp_result.txt | awk '{ print $2 }'` " >> morphing_result.txt

echo -n	"`grep "translational_error.mean" nicp_result_33.txt | awk '{ print $2 }'` " >> morphing_result.txt
echo -n	"`grep "translational_error.std" nicp_result_33.txt | awk '{ print $2 }'` " >> morphing_result.txt
echo -n "`grep "rotational_error.mean" nicp_result_33.txt | awk '{ print $2 }'` " >> morphing_result.txt
echo 	"`grep "rotational_error.std" nicp_result_33.txt | awk '{ print $2 }'` " >> morphing_result.txt

echo -n	"`grep "translational_error.mean" nicp_result_66.txt | awk '{ print $2 }'` " >> morphing_result.txt
echo -n	"`grep "translational_error.std" nicp_result_66.txt | awk '{ print $2 }'` " >> morphing_result.txt
echo -n	"`grep "rotational_error.mean" nicp_result_66.txt | awk '{ print $2 }'` " >> morphing_result.txt
echo 	"`grep "rotational_error.std" nicp_result_66.txt | awk '{ print $2 }'` " >> morphing_result.txt

echo -n	"`grep "translational_error.mean" nicp_result.txt | awk '{ print $2 }'` " >> morphing_result.txt
echo -n	"`grep "translational_error.std" nicp_result.txt | awk '{ print $2 }'` " >> morphing_result.txt
echo -n	"`grep "rotational_error.mean" nicp_result.txt | awk '{ print $2 }'` " >> morphing_result.txt
echo 	"`grep "rotational_error.std" nicp_result.txt | awk '{ print $2 }'` " >> morphing_result.txt

echo "`cat morphing_result.txt`"

