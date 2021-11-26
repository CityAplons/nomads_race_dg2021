#!/usr/bin/env python3
# coding=utf8

from disciplines.afs import afs_init
from disciplines.cargo import cargo_init
from disciplines.formation import formation_init
from disciplines.race import race_init
from disciplines.nrace import nrace_init
from disciplines.grace import grace_init
from disciplines.exp_race import exp_init
import rospy


def main():
    rospy.init_node("solver_node")

    chosen_league = ""
    drone_count = 1
    pids = None

    if rospy.has_param('~pids'):
        rospy.loginfo("Using custom autopilot parameters!")
        pids = rospy.get_param("~pids")

    if not rospy.has_param('~league'):
        rospy.logerr("Specify the league to participate. Halting...")
        return
    else:
        chosen_league = rospy.get_param("~league")
        rospy.loginfo("League: %s", chosen_league)

    if not rospy.has_param('~count'):
        rospy.logwarn("Drone count not specified, using default (1).")
        return
    else:
        drone_count = rospy.get_param("~count")
        rospy.loginfo("Drone count: %d", drone_count)

    leagues = {
        "formation": formation_init,
        "race": race_init,
        "grace": grace_init,
        "nrace": nrace_init,
        "exp_race": exp_init,
        "afs": afs_init,
        "cargo": cargo_init
    }

    if chosen_league in leagues:
        leagues[chosen_league](drone_count, pids)
    else:
        rospy.logerr("League name typo.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
