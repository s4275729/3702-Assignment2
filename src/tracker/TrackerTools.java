package tracker;

import game.AgentState;
import game.RectRegion;
import game.SensingParameters;
import geom.GeomTools;

import java.awt.geom.Point2D;
import java.util.List;

import target.TargetPolicy;
import divergence.MotionHistory;
import divergence.MotionHistory.HistoryEntry;

public class TrackerTools {

	/**
	 * Utility/reward function
	 * 
	 * @param trackerState
	 * @param targetState
	 * @param targetSense
	 * @param trackerSense
	 * @param obstacles
	 * @return
	 */
	public static double utility(AgentState trackerState,
			AgentState targetState, SensingParameters targetSense,
			SensingParameters trackerSense, List<RectRegion> obstacles) {
		// System.out.println(trackerSense.getAngle());
		double reward = 0;
		if (GeomTools.canSee(trackerState, targetState, trackerSense,
				obstacles, 1e-5, 1000)) {
			reward += 1;
		}

		if (GeomTools.canSee(targetState, trackerState, targetSense, obstacles,
				1e-5, 1000)) {
			reward -= 1;
		}

		reward += (1 - getDistanceToTarget(trackerState, targetState));

		return reward;
	}

	public static void maximumUtility(int depth, TargetPolicy targetPolicy,
			AgentState targetState, AgentState trackerState) {
		//get a list of possible actions
		
		//choose the action with maximum utility (taking into account the next target step)
		
		
	}

	/**
	 * Get distance between targets
	 * 
	 * @param trackerState
	 * @param targetState
	 * @return
	 */
	public static double getDistanceToTarget(AgentState trackerState,
			AgentState targetState) {
		Point2D trackerPos = trackerState.getPosition();
		Point2D targetPos = targetState.getPosition();

		return trackerPos.distance(targetPos);
	}

	/**
	 * Get angle between targets
	 * 
	 * @param trackerState
	 * @param targetState
	 * @return
	 */
	public static double getAngleToTarget(AgentState trackerState,
			AgentState targetState) {
		Point2D trackerPos = trackerState.getPosition();
		Point2D targetPos = targetState.getPosition();

		double trackerX = trackerPos.getX();
		double trackerY = trackerPos.getY();
		double targetX = targetPos.getX();
		double targetY = targetPos.getY();

		// 'Move' the target as if the tracker is on (0,0)
		targetX = targetX - trackerX;
		targetY = targetY - trackerY;

		return Math.atan2(targetY, targetX);
	}

	/**
	 * Provide an array of probability given the desired action of the target
	 * and its motion history
	 * 
	 * @param mh
	 * @param desiredAction
	 * @return
	 */
	public static double[] getDivergenceProbability(MotionHistory mh,
			int desiredAction) {
		List<HistoryEntry> history = mh.getHistory();
		double[] probabilities = new double[9];

		int count = 0;
		for (HistoryEntry entry : history) {
			if (entry.getDesiredActionCode() == desiredAction) {
				int resultAction = entry.getResultCode();
				probabilities[resultAction]++;
				count++;
			}
		}

		if (count != 0) {
			for (int i = 0; i < probabilities.length; i++) {
				probabilities[i] = probabilities[i] / count;
			}
		} else {
			return null;
		}
		return probabilities;
	}
}
