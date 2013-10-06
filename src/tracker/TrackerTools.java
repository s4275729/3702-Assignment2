package tracker;

import game.AgentState;
import game.RectRegion;
import game.SensingParameters;
import game.TrackerAction;
import geom.GeomTools;
import geom.GridCell;
import geom.TargetGrid;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.HashMap;
import java.util.List;
import java.util.Random;
import java.util.Set;

import target.TargetPolicy;
import divergence.MotionHistory;
import divergence.MotionHistory.HistoryEntry;

public class TrackerTools {

	public static TrackerAction a;
	final double[] angles = { 112.5, 90, 67.5, 157.5, 135, 45, 22.5, 180, 0,
			202.5, 225, 315, 337.5, 247.5, 270, 292.5 };
	final int[] keys = { 1, 2, 3, 5, 6, 8, 9, 10, 14, 15, 16, 18, 19, 21, 22,
			23 };
	private static int PLANNING_HORIZON = 3;

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
			SensingParameters trackerSense, List<RectRegion> obstacles,
			RectRegion goalRegion) {
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

		if (goalRegion.getRect().contains(trackerState.getPosition())
				&& reward < 1) {
			// dont want to be entering the goal
			reward -= 0.5;
		}
		return reward;
	}

	public static TrackerAction rolloutPlanning(int numberOfTimes,
			AgentState targetState, AgentState trackerState,
			TargetPolicy targetPolicy, MotionHistory targetMotionHistory,
			MotionHistory trackerMotionHistory, SensingParameters targetSense,
			SensingParameters trackerSense, List<RectRegion> obstacles,
			RectRegion goalRegion) {
		MDPState root = new MDPState(targetState, trackerState);

		for (int i = 0; i < numberOfTimes; i++) {
			generateATrace(0, root, targetPolicy, targetMotionHistory,
					trackerMotionHistory, trackerSense, trackerSense,
					obstacles, goalRegion);
		}
		// System.out.println(root.children.get(0).children.get(0).getVisited());
		System.out.println("Utility:" + root.getMaxValue());
		return getPossibleActions(root.getTrackerState(),
				targetPolicy.getGrid()).get(root.getAction());
	}

	public static double generateATrace(int planningHorizon,
			MDPState currentState, TargetPolicy targetPolicy,
			MotionHistory targetMotionHistory,
			MotionHistory trackerMotionHistory, SensingParameters targetSense,
			SensingParameters trackerSense, List<RectRegion> obstacles,
			RectRegion goalRegion) {
		if (planningHorizon > PLANNING_HORIZON)
			return 0;
		currentState.setVisited(currentState.getVisited() + 1);

		// select an action
		HashMap<Integer, TrackerAction> actionmap = getFeasibleActions(
				currentState.getTrackerState(), currentState.getTargetState(), targetPolicy.getGrid());

		Random r = new Random();
		int random = r.nextInt(actionmap.keySet().size());
		int count = 0;
		double maxValue = Double.MIN_VALUE;
		int action = 0;
		for (Integer key : actionmap.keySet()) {
			if (count == random) {
				// select an action uniformly at random
				//action = key;
			}

			// select an action via multiarm bandit
			if (!currentState.actionsPerformed.containsKey(key)) {
				currentState.actionsPerformed.put(key, 1);
			}

			if (!currentState.rewardActions.containsKey(key)) {
				AgentState nextTrackerState = getNextTrackerState(
						currentState.getTrackerState(), key, targetPolicy.getGrid());

				MDPState nextState = new MDPState(targetPolicy.getAction(
						currentState.getTargetState()).getResultingState(),
						nextTrackerState);

				double r_sa = targetUtility(targetMotionHistory,
						nextState.getTargetState(), nextState.getTrackerState(),
						targetPolicy, trackerSense, trackerSense, obstacles, goalRegion);
				currentState.setRewardAction(key, r_sa);
			}
			currentState.updateValue(key);
			
			double traceValue = Math.sqrt(((2 * Math.log(currentState
					.getVisited())) / currentState.actionsPerformed.get(key))) + currentState.valueActions.get(key);

			if (maxValue < traceValue) {
				maxValue = traceValue;
				action = key;
			}
			count++;
		}

		// update actions performed.
		currentState.actionsPerformed.put(action,
				currentState.actionsPerformed.get(action) + 1);

		// System.out.println(currentState.actionsPerformed.entrySet());
		AgentState nextTrackerState = getNextTrackerState(
				currentState.getTrackerState(), action, targetPolicy.getGrid());

		MDPState nextState = new MDPState(targetPolicy.getAction(
				currentState.getTargetState()).getResultingState(),
				nextTrackerState);

		double r_sa = targetUtility(targetMotionHistory,
				nextState.getTargetState(), nextState.getTrackerState(),
				targetPolicy, trackerSense, trackerSense, obstacles, goalRegion);
		currentState.setRewardAction(action, r_sa);

		// sample a next state according to T(s,a,s')
		double[] probabilities = getTrackerDivergenceProbability(
				trackerMotionHistory, action, currentState.getTrackerState(),
				targetPolicy.getGrid(), obstacles);

		int divergedAction = action;
		if (probabilities != null) {
			divergedAction = simulateStateByProbability(probabilities);
			if (divergedAction == -1) {
				// no divergence
				divergedAction = action;
				probabilities = null;
			}
		}

		AgentState divergedTrackerState = getNextTrackerState(
				currentState.getTrackerState(), divergedAction,
				targetPolicy.getGrid());
		MDPState divergedState = new MDPState(targetPolicy.getAction(
				currentState.getTargetState()).getResultingState(),
				divergedTrackerState);

		divergedState.setParentActionCode(action);
		divergedState.setDepth(planningHorizon);
		// add s' as a child
		if (!currentState.childExists(divergedState)) {
			currentState.addChild(divergedState);
			if (probabilities != null) {
				divergedState.setProbability(probabilities[divergedAction]);
			} else {
				divergedState.setProbability(1.0);
			}
		} else {
			divergedState = currentState.getChild(divergedState);
		}

		generateATrace(planningHorizon + 1, divergedState, targetPolicy,
				targetMotionHistory, trackerMotionHistory, targetSense,
				trackerSense, obstacles, goalRegion);

		currentState.updateValue(action);
		return 0;
	}

	public static double targetUtility(MotionHistory targetMotionHistory,
			AgentState targetState, AgentState trackerState,
			TargetPolicy targetPolicy, SensingParameters targetSense,
			SensingParameters trackerSense, List<RectRegion> obstacles,
			RectRegion goalRegion) {
		game.Action expectedAction = targetPolicy.getAction(targetState);
		TargetGrid grid = targetPolicy.getGrid();
		double[] probs = getTargetDivergenceProbability(targetMotionHistory,
				grid.encodeAction(expectedAction), grid, targetState, obstacles);

		double sum = 0;
		for (int i = 0; i < 9; i++) {
			if (probs[i] != 0) {
				GridCell nextCell = grid.decodeFromIndices(
						grid.getCell(targetState.getPosition()), i);
				AgentState resultTargetState = new AgentState(
						grid.getCentre(nextCell), i);

				// Calculate the utility of the resulting tracker state
				// and
				// resulting target state
				double utility = probs[i]
						* utility(trackerState, resultTargetState, targetSense,
								trackerSense, obstacles, goalRegion);
				sum += utility;
			}
		}
		sum += utility(trackerState, targetState, targetSense,
				trackerSense, obstacles, goalRegion);
		return sum;
	}

	public static int simulateStateByProbability(double[] probabilities) {
		int action = -1;
		double sum = 0;
		Random r = new Random();
		double random = r.nextDouble();

		for (int i = 0; i < 25; i++) {
			if (probabilities[i] == 0)
				continue;
			if (random >= sum && random < sum + probabilities[i]) {
				action = i;
				break;

			}
			sum = sum + probabilities[i];
		}

		if (action == -1) {
			//
		}
		return action;
	}

	public static double maxUtility(int depth, TargetPolicy targetPolicy,
			AgentState targetState, MotionHistory targetMotionHistory,
			AgentState trackerState, SensingParameters targetSense,
			SensingParameters trackerSense, List<RectRegion> obstacles,
			RectRegion goalRegion) {

		TargetGrid grid = targetPolicy.getGrid();

		if (depth == 0) {
			return utility(trackerState, targetState, targetSense,
					trackerSense, obstacles, goalRegion);
		}

		else {
			double maxUtility = 0.0;
			Integer maxActionKey = 1;

			// get a list of possible actions
			HashMap<Integer, TrackerAction> actions = getPossibleActions(
					trackerState, grid);
			Set<Integer> actionSet = actions.keySet();

			// System.out.print(targetState);
			// System.out.println(targetPolicy.getAction(targetState).getResultingState());

			double[] probs = TrackerTools.getTargetDivergenceProbability(
					targetMotionHistory,
					grid.encodeAction(targetPolicy.getAction(targetState)),
					grid, targetState, obstacles);

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
								grid.getCentre(nextCell), grid.getHeading(i));

						// Calculate the utility of the resulting tracker state
						// and
						// resulting target state
						double utility = probs[i]
								* (maxUtility(depth - 1, targetPolicy,
										resultTargetState, targetMotionHistory,
										resultTrackerState, targetSense,
										trackerSense, obstacles, goalRegion) + utility(
										resultTrackerState, targetState,
										targetSense, trackerSense, obstacles,
										goalRegion));

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
	 * @param targetState
	 * @param obstacles
	 * @return
	 */
	public static double[] getTargetDivergenceProbability(MotionHistory mh,
			int desiredAction, TargetGrid grid, AgentState targetState,
			List<RectRegion> obstacles) {
		List<HistoryEntry> history = mh.getHistory();
		double[] resultActionCount = new double[9];

		int count = 0;
		for (HistoryEntry entry : history) {
			if (entry.getDesiredActionCode() == desiredAction) {
				int resultAction = entry.getResultCode();
				resultActionCount[resultAction]++;
				count++;
			}
		}

		double[] probabilities = new double[9];
		if (count != 0) {
			for (int i = 0; i < resultActionCount.length; i++) {
				probabilities[i] += resultActionCount[i] / count;

				if (probabilities[i] > 0) {
					GridCell nextCell = grid.decodeFromIndices(
							grid.getCell(targetState.getPosition()), i);
					AgentState resultTargetState = new AgentState(
							grid.getCentre(nextCell), grid.getHeading(i));

					boolean hitObstacles = !GeomTools.canMove(
							targetState.getPosition(),
							resultTargetState.getPosition(),
							targetState.hasCamera(),
							targetState.getCameraArmLength(), obstacles);

					boolean outOfBounds = resultTargetState.getPosition()
							.getX() < 0
							|| resultTargetState.getPosition().getY() < 0;

					if (outOfBounds || hitObstacles) {
						probabilities[4] += probabilities[i];
						probabilities[i] = 0;
					}
				}
			}
		} else {
			return null;
		}
		return probabilities;
	}

	public static double[] getTrackerDivergenceProbability(MotionHistory mh,
			int desiredAction, AgentState trackerState, TargetGrid grid,
			List<RectRegion> obstacles) {
		List<HistoryEntry> history = mh.getHistory();

		double[] actionCount = new double[25];

		int count = 0;
		for (HistoryEntry entry : history) {
			if (entry.getDesiredActionCode() == desiredAction) {
				int resultAction = entry.getResultCode();
				actionCount[resultAction]++;
				count++;
			}
		}

		double[] probabilities = new double[25];
		if (count != 0) {
			for (int i = 0; i < actionCount.length; i++) {
				probabilities[i] += actionCount[i] / count;

				if (probabilities[i] > 0) {
					AgentState nextTrackerState = getNextTrackerState(
							trackerState, i, grid);

					boolean hitObstacles = !GeomTools.canMove(
							trackerState.getPosition(),
							nextTrackerState.getPosition(),
							trackerState.hasCamera(),
							trackerState.getCameraArmLength(), obstacles);

					boolean outOfBounds = nextTrackerState.getPosition().getX() < 0
							|| nextTrackerState.getPosition().getY() < 0;

					if (outOfBounds || hitObstacles) {

						probabilities[12] += probabilities[i];
						probabilities[i] = 0;
					}
				}
			}
		} else {
			return null;
		}
		return probabilities;
	}

	public static AgentState getNextTrackerState(
			AgentState currentTrackerState, int actionKey, TargetGrid grid) {
		double distance = 1.0 / grid.getGridSize();
		HashMap<Integer, TrackerAction> actionsList = getPossibleActions(
				currentTrackerState, grid);

		TrackerAction selectedAction = actionsList.get(actionKey);

		if (selectedAction == null) {
			TrackerAction similarAction;

			switch (actionKey) {
			case 0:
				similarAction = actionsList.get(6);
				selectedAction = new TrackerAction(currentTrackerState,
						similarAction.getHeading(), (2 * distance));
				break;
			case 4:
				similarAction = actionsList.get(8);
				selectedAction = new TrackerAction(currentTrackerState,
						similarAction.getHeading(), (2 * distance));
				break;
			case 7:
				similarAction = actionsList.get(2);
				selectedAction = new TrackerAction(currentTrackerState,
						similarAction.getHeading(), (distance / 2));
				break;
			case 11:
				similarAction = actionsList.get(10);
				selectedAction = new TrackerAction(currentTrackerState,
						similarAction.getHeading(), (distance / 2));
				break;
			case 13:
				similarAction = actionsList.get(14);
				selectedAction = new TrackerAction(currentTrackerState,
						similarAction.getHeading(), (distance / 2));
				break;
			case 17:
				similarAction = actionsList.get(22);
				selectedAction = new TrackerAction(currentTrackerState,
						similarAction.getHeading(), (distance / 2));
				break;
			case 20:
				similarAction = actionsList.get(16);
				selectedAction = new TrackerAction(currentTrackerState,
						similarAction.getHeading(), (2 * distance));
				break;
			case 24:
				similarAction = actionsList.get(18);
				selectedAction = new TrackerAction(currentTrackerState,
						similarAction.getHeading(), (2 * distance));
				break;
			}
		}

		return selectedAction.getResultingState();
	}

	public static HashMap<Integer, TrackerAction> getFeasibleActions(
			AgentState trackerState, AgentState targetState, TargetGrid grid) {
		HashMap<Integer, TrackerAction> feasibleActions = new HashMap<Integer, TrackerAction>();
		HashMap<Integer, TrackerAction> possibleActions = getPossibleActions(
				trackerState, grid);

		double angleToTarget = getAngleToTarget(trackerState, targetState);
		double range = Math.toRadians(90);

		Set<Integer> keySet = possibleActions.keySet();

		for (Integer key : keySet) {
			double actionHeading = possibleActions.get(key).getHeading();

			if (actionHeading > (angleToTarget - range)
					|| actionHeading < (angleToTarget + range)) {
				feasibleActions.put(key, possibleActions.get(key));
			}
		}

		return feasibleActions;
	}

	public static HashMap<Integer, TrackerAction> getPossibleActions(
			AgentState currentTrackerState, TargetGrid grid) {
		HashMap<Integer, TrackerAction> possibleActions = new HashMap<Integer, TrackerAction>();
		double distance = 1.0 / grid.getGridSize();

		possibleActions.put(
				1,
				new TrackerAction(currentTrackerState, GeomTools
						.normaliseAngle(Math.toRadians(112.5)), distance));
		possibleActions.put(
				2,
				new TrackerAction(currentTrackerState, GeomTools
						.normaliseAngle(Math.toRadians(90)), distance));
		possibleActions.put(
				3,
				new TrackerAction(currentTrackerState, GeomTools
						.normaliseAngle(Math.toRadians(67.5)), distance));
		possibleActions.put(
				5,
				new TrackerAction(currentTrackerState, GeomTools
						.normaliseAngle(Math.toRadians(157.5)), distance));
		possibleActions.put(
				6,
				new TrackerAction(currentTrackerState, GeomTools
						.normaliseAngle(Math.toRadians(135)), distance));
		possibleActions.put(
				8,
				new TrackerAction(currentTrackerState, GeomTools
						.normaliseAngle(Math.toRadians(45)), distance));
		possibleActions.put(
				9,
				new TrackerAction(currentTrackerState, GeomTools
						.normaliseAngle(Math.toRadians(22.5)), distance));
		possibleActions.put(10, new TrackerAction(currentTrackerState,
				GeomTools.normaliseAngle(Math.toRadians(180)), distance));
		possibleActions.put(12, new TrackerAction(currentTrackerState,
				currentTrackerState.getHeading(), 0));
		possibleActions.put(14, new TrackerAction(currentTrackerState,
				GeomTools.normaliseAngle(Math.toRadians(0)), distance));
		possibleActions.put(15, new TrackerAction(currentTrackerState,
				GeomTools.normaliseAngle(Math.toRadians(202.5)), distance));
		possibleActions.put(16, new TrackerAction(currentTrackerState,
				GeomTools.normaliseAngle(Math.toRadians(225)), distance));
		possibleActions.put(18, new TrackerAction(currentTrackerState,
				GeomTools.normaliseAngle(Math.toRadians(315)), distance));
		possibleActions.put(19, new TrackerAction(currentTrackerState,
				GeomTools.normaliseAngle(Math.toRadians(337.5)), distance));
		possibleActions.put(21, new TrackerAction(currentTrackerState,
				GeomTools.normaliseAngle(Math.toRadians(247.5)), distance));
		possibleActions.put(22, new TrackerAction(currentTrackerState,
				GeomTools.normaliseAngle(Math.toRadians(270)), distance));
		possibleActions.put(23, new TrackerAction(currentTrackerState,
				GeomTools.normaliseAngle(Math.toRadians(292.5)), distance));

		possibleActions.put(121, new TrackerAction(currentTrackerState,
				GeomTools.normaliseAngle(Math.toRadians(112.5)), 0));
		possibleActions.put(122, new TrackerAction(currentTrackerState,
				GeomTools.normaliseAngle(Math.toRadians(90)), 0));
		possibleActions.put(123, new TrackerAction(currentTrackerState,
				GeomTools.normaliseAngle(Math.toRadians(67.5)), 0));
		possibleActions.put(125, new TrackerAction(currentTrackerState,
				GeomTools.normaliseAngle(Math.toRadians(157.5)), 0));
		possibleActions.put(126, new TrackerAction(currentTrackerState,
				GeomTools.normaliseAngle(Math.toRadians(135)), 0));
		possibleActions.put(128, new TrackerAction(currentTrackerState,
				GeomTools.normaliseAngle(Math.toRadians(45)), 0));
		possibleActions.put(129, new TrackerAction(currentTrackerState,
				GeomTools.normaliseAngle(Math.toRadians(22.5)), 0));
		possibleActions.put(1210, new TrackerAction(currentTrackerState,
				GeomTools.normaliseAngle(Math.toRadians(180)), 0));
		possibleActions.put(1214, new TrackerAction(currentTrackerState,
				GeomTools.normaliseAngle(Math.toRadians(0)), 0));
		possibleActions.put(1215, new TrackerAction(currentTrackerState,
				GeomTools.normaliseAngle(Math.toRadians(202.5)), 0));
		possibleActions.put(1216, new TrackerAction(currentTrackerState,
				GeomTools.normaliseAngle(Math.toRadians(225)), 0));
		possibleActions.put(1218, new TrackerAction(currentTrackerState,
				GeomTools.normaliseAngle(Math.toRadians(315)), 0));

		possibleActions.put(1219, new TrackerAction(currentTrackerState,
				GeomTools.normaliseAngle(Math.toRadians(337.5)), 0));
		possibleActions.put(1221, new TrackerAction(currentTrackerState,
				GeomTools.normaliseAngle(Math.toRadians(247.5)), 0));

		possibleActions.put(1222, new TrackerAction(currentTrackerState,
				GeomTools.normaliseAngle(Math.toRadians(270)), 0));

		possibleActions.put(1223, new TrackerAction(currentTrackerState,
				GeomTools.normaliseAngle(Math.toRadians(292.5)), 0));

		return possibleActions;
	}

}
