package tracker;

import game.AgentState;
import game.RectRegion;
import game.SensingParameters;
import game.TrackerAction;
import geom.GeomTools;
import geom.GridCell;
import geom.TargetGrid;

import java.awt.geom.Point2D;
import java.util.HashMap;
import java.util.List;
import java.util.Set;

import target.TargetPolicy;
import divergence.MotionHistory;
import divergence.MotionHistory.HistoryEntry;

public class TrackerTools {

	public static TrackerAction a;

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

	public static double maxUtility(int depth, TargetPolicy targetPolicy,
			AgentState targetState, double[] probs, AgentState trackerState,
			SensingParameters targetSense, SensingParameters trackerSense,
			List<RectRegion> obstacles) {

		TargetGrid grid = targetPolicy.getGrid();

		if (depth == 0) {
			return utility(trackerState, targetState, targetSense,
					trackerSense, obstacles);
		}

		else {
			double maxUtility = 0.0;
			Integer maxActionKey = 1;

			// get a list of possible actions
			HashMap<Integer, TrackerAction> actions = getPossibleActions(
					trackerState, grid);
			Set<Integer> actionSet = actions.keySet();

			for (Integer key : actionSet) {
				TrackerAction act = actions.get(key);
				AgentState resultTrackerState = act.getResultingState();

				double totalUtilityForEachTrackerMove = 0;

				// Iterate through the possible next states of the target
				for (int i = 0; i < probs.length; i++) {
					// If the target has a probability to do action i
					if (probs[i] != 0) {
						GridCell nextCell = grid.decodeFromIndices(
								grid.getCell(targetState.getPosition()), i);
						AgentState resultTargetState = new AgentState(
								grid.getCentre(nextCell), i);

						// Calculate the utility of the resulting tracker state
						// and
						// resulting target state
						double utility = probs[i]
								* (maxUtility(depth - 1, targetPolicy,
										resultTargetState, probs,
										resultTrackerState, targetSense,
										trackerSense, obstacles) + utility(
										resultTrackerState, resultTargetState, targetSense,
										trackerSense, obstacles));

						// Sum up all the utilities of each possible target
						// states
						totalUtilityForEachTrackerMove += utility;
					}
				}

				if (totalUtilityForEachTrackerMove > maxUtility) {
					maxUtility = totalUtilityForEachTrackerMove;
					maxActionKey = key;
				}

			}
			a = actions.get(maxActionKey);
			return maxUtility;
		}
		// choose the action with maximum utility (taking into account the next
		// target step)

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

	public static HashMap<Integer, TrackerAction> getPossibleActions(
			AgentState currentTrackerState, TargetGrid grid) {
		HashMap<Integer, TrackerAction> possibleActions = new HashMap<Integer, TrackerAction>();
		double distance = 1.0 / grid.getGridSize();

		possibleActions.put(1,
				new TrackerAction(currentTrackerState, Math.toRadians(112.5),
						distance));
		possibleActions.put(2,
				new TrackerAction(currentTrackerState, Math.toRadians(90),
						distance));
		possibleActions.put(3,
				new TrackerAction(currentTrackerState, Math.toRadians(67.5),
						distance));
		possibleActions.put(5,
				new TrackerAction(currentTrackerState, Math.toRadians(157.5),
						distance));
		possibleActions.put(6,
				new TrackerAction(currentTrackerState, Math.toRadians(135),
						distance));
		possibleActions.put(8,
				new TrackerAction(currentTrackerState, Math.toRadians(45),
						distance));
		possibleActions.put(9,
				new TrackerAction(currentTrackerState, Math.toRadians(22.5),
						distance));
		possibleActions.put(10,
				new TrackerAction(currentTrackerState, Math.toRadians(180),
						distance));
		possibleActions.put(12, new TrackerAction(currentTrackerState, 0, 0));
		possibleActions.put(14,
				new TrackerAction(currentTrackerState, Math.toRadians(0),
						distance));
		possibleActions.put(15,
				new TrackerAction(currentTrackerState, Math.toRadians(202.5),
						distance));
		possibleActions.put(16,
				new TrackerAction(currentTrackerState, Math.toRadians(225),
						distance));
		possibleActions.put(18,
				new TrackerAction(currentTrackerState, Math.toRadians(315),
						distance));
		possibleActions.put(19,
				new TrackerAction(currentTrackerState, Math.toRadians(337.5),
						distance));
		possibleActions.put(21,
				new TrackerAction(currentTrackerState, Math.toRadians(247.5),
						distance));
		possibleActions.put(22,
				new TrackerAction(currentTrackerState, Math.toRadians(270),
						distance));
		possibleActions.put(23,
				new TrackerAction(currentTrackerState, Math.toRadians(292.5),
						distance));

		possibleActions
				.put(121,
						new TrackerAction(currentTrackerState, Math
								.toRadians(112.5), 0));
		possibleActions.put(122,
				new TrackerAction(currentTrackerState, Math.toRadians(90), 0));
		possibleActions
				.put(123,
						new TrackerAction(currentTrackerState, Math
								.toRadians(67.5), 0));
		possibleActions
				.put(125,
						new TrackerAction(currentTrackerState, Math
								.toRadians(157.5), 0));
		possibleActions.put(126,
				new TrackerAction(currentTrackerState, Math.toRadians(135), 0));
		possibleActions.put(128,
				new TrackerAction(currentTrackerState, Math.toRadians(45), 0));
		possibleActions
				.put(129,
						new TrackerAction(currentTrackerState, Math
								.toRadians(22.5), 0));
		possibleActions.put(1210,
				new TrackerAction(currentTrackerState, Math.toRadians(180), 0));
		possibleActions
				.put(1215,
						new TrackerAction(currentTrackerState, Math
								.toRadians(202.5), 0));
		possibleActions.put(1216,
				new TrackerAction(currentTrackerState, Math.toRadians(225), 0));
		possibleActions.put(1218,
				new TrackerAction(currentTrackerState, Math.toRadians(315), 0));
		possibleActions
				.put(1219,
						new TrackerAction(currentTrackerState, Math
								.toRadians(337.5), 0));
		possibleActions
				.put(1221,
						new TrackerAction(currentTrackerState, Math
								.toRadians(247.5), 0));
		possibleActions.put(1222,
				new TrackerAction(currentTrackerState, Math.toRadians(270), 0));
		possibleActions
				.put(1223,
						new TrackerAction(currentTrackerState, Math
								.toRadians(292.5), 0));

		return possibleActions;
	}
}
