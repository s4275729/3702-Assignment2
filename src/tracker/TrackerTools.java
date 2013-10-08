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
	private static int PLANNING_HORIZON = 1;

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
			reward -= 0.7;
		}
		/*
		 * double angleToTarget= getAngleToTarget(trackerState, targetState);
		 * double range = Math.toRadians(45); if (trackerState.getHeading() <
		 * (angleToTarget + (0.5 * range)) && trackerState.getHeading() >
		 * (angleToTarget - (0.5 * range))) { reward += 0.3; //want to be
		 * pointing at the target }
		 */
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
					trackerMotionHistory, targetSense, trackerSense, obstacles,
					goalRegion);
		}
		// System.out.println(root.children.get(0).children.get(0).getVisited());
		System.out.println("Action Code: " + root.getAction());
		return getAllPossibleActions(root.getTrackerState(),
				targetPolicy.getGrid(), trackerSense).get(root.getAction());
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

		TargetGrid grid = targetPolicy.getGrid();
		// select an action
		HashMap<Integer, TrackerAction> actionmap = getAllPossibleActions(
				currentState.getTrackerState(), grid, trackerSense);

		Random r = new Random();
		int random = r.nextInt(actionmap.keySet().size());
		int count = 0;
		double maxValue = Double.MIN_VALUE;
		int action = 0;
		for (Integer key : actionmap.keySet()) {
			if (count == random) {
				// select an action uniformly at random
				// action = key;
			}

			if (!currentState.actionsPerformed.containsKey(key)) {
				currentState.actionsPerformed.put(key, 1);
			}

			// select an action via multiarm bandit if
			if (!currentState.rewardActions.containsKey(key)) {
				AgentState nextTrackerState = getNextTrackerState(
						currentState.getTrackerState(), key, grid, trackerSense);

				MDPState nextState = new MDPState(targetPolicy.getAction(
						currentState.getTargetState()).getResultingState(),
						nextTrackerState);

				double r_sa = MDPUtility(targetMotionHistory, nextState,
						targetPolicy, trackerSense, trackerSense, obstacles,
						goalRegion);
				currentState.setRewardAction(key, r_sa);
			}
			currentState.updateValue(key);

			double traceValue = Math.sqrt(((2 * Math.log(currentState
					.getVisited())) / currentState.actionsPerformed.get(key)))
					+ currentState.valueActions.get(key);

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
				currentState.getTrackerState(), action, grid, trackerSense);
		MDPState nextState = new MDPState(currentState.getTargetState(),
				nextTrackerState);

		double r_sa = MDPUtility(targetMotionHistory, nextState, targetPolicy,
				trackerSense, trackerSense, obstacles, goalRegion);
		currentState.setRewardAction(action, r_sa);

		// sample a next state according to T(s,a,s')
		double[] probabilities = null;
		
		if (trackerMotionHistory != null) {
			probabilities = getTrackerDivergenceProbability(
		 trackerMotionHistory, action, currentState.getTrackerState(),
		 targetPolicy.getGrid(), obstacles, trackerSense);
		}
		 

		// simulate diverging tracker state
		int divergedAction = action;
		double divergedActionProbability = 0;
		if (probabilities != null) {
			divergedAction = simulateStateByProbability(probabilities);
			if (divergedAction == -1) {
				// no divergence
				divergedAction = action;
				divergedActionProbability = 1;
				probabilities = null;
			} else {
				divergedActionProbability = probabilities[divergedAction];
			}
		} else {
			divergedActionProbability = 1;
		}
		AgentState divergedTrackerState = getNextTrackerState(
				currentState.getTrackerState(), divergedAction, grid, trackerSense);

		// simulate diverging target state

		double divergedTargetProbability = 0;
		int divergedTargetAction = grid.encodeAction(targetPolicy
				.getAction(currentState.getTargetState()));
		double[] targetProbabilities = getTargetDivergenceProbability(
				targetMotionHistory, divergedTargetAction, grid,
				currentState.getTargetState(), obstacles);

		if (targetProbabilities != null) {
			divergedTargetAction = simulateStateByProbability(targetProbabilities);
			if (divergedAction == -1) {
				// no divergence
				divergedTargetAction = targetPolicy.getGrid().encodeAction(
						targetPolicy.getAction(currentState.getTargetState()));
				divergedTargetProbability = 1;
				targetProbabilities = null;
			} else {
				divergedTargetProbability = targetProbabilities[divergedTargetAction];
			}
		} else {
			divergedTargetProbability = 1;
		}

		GridCell nextCell = grid.decodeFromIndices(
				grid.getCell(currentState.getTargetState().getPosition()),
				divergedTargetAction);
		AgentState divergedTargetState = new AgentState(
				grid.getCentre(nextCell), grid.getHeading(divergedTargetAction));
		// AgentState divergedTargetState =
		// targetPolicy.getAction(currentState.getTargetState()).getResultingState();
		MDPState divergedState = new MDPState(divergedTargetState,
				divergedTrackerState);

		divergedState.setParentActionCode(action);
		divergedState.setDepth(planningHorizon);
		// add s' as a child
		if (!currentState.childExists(divergedState)) {
			currentState.addChild(divergedState);
			divergedState.setProbability(divergedActionProbability
					* divergedTargetProbability);
			// System.out.print("PROB: " + divergedActionProbability *
			// divergedTargetProbability);
			// divergedState.setTargetProbability(divergedTargetProbability);

		} else {
			divergedState = currentState.getChild(divergedState);
		}

		generateATrace(planningHorizon + 1, divergedState, targetPolicy,
				targetMotionHistory, trackerMotionHistory, targetSense,
				trackerSense, obstacles, goalRegion);

		currentState.updateValue(action);
		/*
		 * System.out.println("horizon" + planningHorizon + " " +
		 * currentState.getMaxValue() + "size: " + currentState.children.size()
		 * + "reward: " + currentState.rewardActions.get(action));
		 */
		return 0;
	}

	public static double MDPUtility(MotionHistory targetMotionHistory,
			MDPState state, TargetPolicy targetPolicy,
			SensingParameters targetSense, SensingParameters trackerSense,
			List<RectRegion> obstacles, RectRegion goalRegion) {

		double sum = 0;

		sum += utility(state.getTrackerState(), state.getTargetState(),
				targetSense, trackerSense, obstacles, goalRegion);

		game.Action expectedAction = targetPolicy.getAction(state
				.getTargetState());
		TargetGrid grid = targetPolicy.getGrid();
		double[] probs = getTargetDivergenceProbability(targetMotionHistory,
				grid.encodeAction(expectedAction), grid,
				state.getTargetState(), obstacles);
		if (probs == null) {
			probs = new double[9];
			probs[grid.encodeAction(targetPolicy.getAction(state
					.getTargetState()))] = 1;
		}
		for (int i = 0; i < 9; i++) {
			if (probs[i] != 0) {
				GridCell nextCell = grid.decodeFromIndices(
						grid.getCell(state.getTargetState().getPosition()), i);
				AgentState resultTargetState = new AgentState(
						grid.getCentre(nextCell), i);

				// Calculate the utility of the resulting tracker state
				// and
				// resulting target state
				double utility = probs[i]
						* utility(state.getTrackerState(), resultTargetState,
								targetSense, trackerSense, obstacles,
								goalRegion);
				sum += utility;
			}
		}
		// sum *= state.getTargetProbability();
		return sum;
	}

	public static int simulateStateByProbability(double[] probabilities) {
		int action = -1;
		double sum = 0;
		Random r = new Random();
		double random = r.nextDouble();

		for (int i = 0; i < probabilities.length; i++) {
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
			HashMap<Integer, TrackerAction> actions = getAllPossibleActions(
					trackerState, grid, trackerSense);
			Set<Integer> actionSet = actions.keySet();

			// System.out.print(targetState);
			// System.out.println(targetPolicy.getAction(targetState).getResultingState());

			double[] probs = TrackerTools.getTargetDivergenceProbability(
					targetMotionHistory,
					grid.encodeAction(targetPolicy.getAction(targetState)),
					grid, targetState, obstacles);

			/*if (probs == null) {
				// probs = new double[9];
				probs[grid.encodeAction(targetPolicy.getAction(targetState))] = 1;
			}*/

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

					boolean outOfBounds = 	resultTargetState.getPosition().getX() < 0
											|| resultTargetState.getPosition().getY() < 0 
											|| resultTargetState.getPosition().getX() > 1
											|| resultTargetState.getPosition().getY() > 1;

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
			List<RectRegion> obstacles, SensingParameters trackerSense) {
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
							trackerState, i, grid, trackerSense);

					boolean hitObstacles = !GeomTools.canMove(
							trackerState.getPosition(),
							nextTrackerState.getPosition(),
							trackerState.hasCamera(),
							trackerState.getCameraArmLength(), obstacles);

					boolean outOfBounds =  nextTrackerState.getPosition().getX() < 0
										|| nextTrackerState.getPosition().getY() < 0 
										|| nextTrackerState.getPosition().getX() > 1
										|| nextTrackerState.getPosition().getY() > 1;
					
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
			AgentState currentTrackerState, int actionKey, TargetGrid grid, SensingParameters trackerSense) {
		double distance = 1.0 / grid.getGridSize();
		HashMap<Integer, TrackerAction> actionsList = getAllPossibleActions(
				currentTrackerState, grid, trackerSense);

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

	public static HashMap<Integer, TrackerAction> getAllFeasibleActions(
			AgentState trackerState, AgentState targetState, TargetGrid grid, SensingParameters trackerSense) {
		
		HashMap<Integer, TrackerAction> feasibleActions = new HashMap<Integer, TrackerAction>();
		feasibleActions.putAll(getFeasibleMovementActions(trackerState, targetState, grid));
		
		if(trackerState.hasCamera())
			feasibleActions.putAll(getFeasibleCameraAdjustmentActions(trackerState, targetState, grid, trackerSense));
		
		return feasibleActions;
	}
	
	public static HashMap<Integer, TrackerAction> getAllPossibleActions(
			AgentState currentTrackerState, TargetGrid grid, SensingParameters trackerSense)
	{
		HashMap<Integer, TrackerAction> allActions = new HashMap<Integer, TrackerAction>();
	
		allActions.putAll(getPossibleMovementActions(currentTrackerState, grid));
		
		if(currentTrackerState.hasCamera())
		{
			allActions.putAll(getPossibleCameraAdjustmentActions(currentTrackerState, trackerSense));
		}
		
		return allActions;
	}
	
	public static HashMap<Integer, TrackerAction> getFeasibleMovementActions(
			AgentState trackerState, AgentState targetState, TargetGrid grid) {
		HashMap<Integer, TrackerAction> feasibleActions = new HashMap<Integer, TrackerAction>();
		HashMap<Integer, TrackerAction> possibleActions = getPossibleMovementActions(
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

	public static HashMap<Integer, TrackerAction> getPossibleMovementActions(
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
	
	public static HashMap<Integer, TrackerAction> getFeasibleCameraAdjustmentActions(
			AgentState trackerState, AgentState targetState, TargetGrid grid, SensingParameters trackerSense) {
		
		HashMap<Integer, TrackerAction> feasibleActions = new HashMap<Integer, TrackerAction>();
		HashMap<Integer, TrackerAction> cameraAdjustmentActions = getPossibleCameraAdjustmentActions(trackerState, trackerSense);

		double angleToTarget = getAngleToTarget(trackerState, targetState);
		double distanceToTarget = getDistanceToTarget(trackerState, targetState);
		double range = Math.toRadians(90);
		
		double angleFromArmToTarget = angleToTarget + (Math.toRadians(90) - trackerState.getHeading());
		double orthogonalTargetDistance = distanceToTarget * Math.cos(angleFromArmToTarget);
		double orthogonalTargetHeight = distanceToTarget * Math.sin(angleFromArmToTarget);
		double currentArmLength = trackerState.getCameraArmLength();
		
		double maxFieldOfView = trackerSense.getMaxLength();
		double minFieldOfView = trackerSense.getMinLength();
		
		//System.out.println("Max FOV: " + maxFieldOfView);
		//System.out.println("Min FOV: " + minFieldOfView);
		// If the target is on the right hand side and inside the possible field view of the camera
		if(angleFromArmToTarget <= range && orthogonalTargetHeight <= trackerSense.getRange() 
		   && orthogonalTargetDistance <= maxFieldOfView && orthogonalTargetDistance >= minFieldOfView)
		{
			Set<Integer> cameraKeyset = cameraAdjustmentActions.keySet();
			
			for(Integer key : cameraKeyset)
			{
				double newLength = cameraAdjustmentActions.get(key).getResultingState().getCameraArmLength();
				
				if(orthogonalTargetDistance <= currentArmLength && newLength <= currentArmLength)
				{
					//System.out.println("SHORTEN, current arm: " + currentArmLength + " Target's orthogonal: " + orthogonalTargetDistance + " new arm: " + newLength);
					feasibleActions.put(key, cameraAdjustmentActions.get(key));
				}
				
				if(orthogonalTargetDistance > currentArmLength && newLength > currentArmLength)
				{
					//System.out.println("LENGTHEN, current arm: " + currentArmLength + " Target's orthogonal: " + orthogonalTargetDistance + " new arm: " + newLength);
					feasibleActions.put(key, cameraAdjustmentActions.get(key));
				}
			}
		}

		return feasibleActions;
	}
	
	public static HashMap<Integer, TrackerAction> getPossibleCameraAdjustmentActions(
			AgentState trackerState, SensingParameters trackerSense) 
	{
		HashMap<Integer, TrackerAction> possibleCameraActions = new HashMap<Integer, TrackerAction>();
		
		double maxArmLength = trackerSense.getMaxLength();
		double minArmLength = trackerSense.getMinLength();
		
		// Discretizes the arm length into 10 possible length
		double step = (maxArmLength - minArmLength) / 10;
		
		boolean currentStateExists = false;
		
		for(int i = 0; i < 11; i++)
		{
			double newArmLength = minArmLength + (step*i);
			possibleCameraActions.put(30+i, new TrackerAction(trackerState, newArmLength));
			
			if(newArmLength == trackerState.getCameraArmLength())
				currentStateExists = true;
		}
		
		if(!currentStateExists)
			possibleCameraActions.put(312, new TrackerAction(trackerState, trackerState.getCameraArmLength()));
		
		return possibleCameraActions;
	}
}
