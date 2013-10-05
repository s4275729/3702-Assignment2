package tracker;

import game.ActionResult;
import game.Agent;
import game.AgentState;
import game.Percept;
import game.RectRegion;
import game.SensingParameters;
import game.TrackerAction;
import geom.GridCell;
import geom.TargetGrid;

import java.awt.geom.Point2D;
import java.util.List;
import java.util.Random;
import divergence.MotionHistory;
import divergence.MotionHistory.HistoryEntry;
import target.TargetPolicy;

public class Tracker implements Agent {
	/** The number of targets. */
	private int numTargets;
	/** The policy of the target(s). */
	private TargetPolicy targetPolicy;
	/**
	 * The motion history of the target(s), or null if no history is available.
	 * */
	private MotionHistory targetMotionHistory;
	/** The sensing parameters of the target(s). */
	private SensingParameters targetSensingParams;
	/** The initial state(s) of the target(s). */
	private List<AgentState> targetInitialStates;

	/** The motion history of this tracker. */
	private MotionHistory myMotionHistory;
	/** The sensing parameters of this tracker. */
	private SensingParameters mySensingParams;
	/** The initial state of this tracker. */
	private AgentState myInitialState;

	/** The obstacles. */
	private List<RectRegion> obstacles;
	/** The goal region. */
	private RectRegion goalRegion;

	/** 
	 * 
	 */
	private GridCell currentTarget;
	private AgentState targetState;

	/**
	 * Constructs a tracker with the given parameters.
	 * 
	 * This gives your tracker all of the information it has about the initial
	 * state of the game, including very important aspects such as the target's
	 * policy, which is known to the tracker.
	 * 
	 * @param numTargets
	 *            the number of targets.
	 * @param targetPolicy
	 *            the policy of the target(s).
	 * @param targetMotionHistory
	 *            the motion history of the target(s), or null if no history is
	 *            available.
	 * @param targetSensingParams
	 *            the sensing parameters of the target(s).
	 * @param targetInitialStates
	 *            the initial state(s) of the target(s).
	 * @param trackerMotionHistory
	 *            the motion history of this tracker.
	 * @param trackerSensingParams
	 *            the sensing parameters of this tracker.
	 * @param trackerInitialState
	 *            the initial state of this tracker.
	 * @param obstacles
	 *            the obstacles.
	 * @param goalRegion
	 *            the goal region.
	 */
	public Tracker(int numTargets, TargetPolicy targetPolicy,
			MotionHistory targetMotionHistory,
			SensingParameters targetSensingParams,
			List<AgentState> targetInitialStates,

			MotionHistory trackerMotionHistory,
			SensingParameters trackerSensingParams,
			AgentState trackerInitialState,

			List<RectRegion> obstacles, RectRegion goalRegion) {
		this.numTargets = numTargets;
		this.targetPolicy = targetPolicy;
		this.targetMotionHistory = targetMotionHistory;
		this.targetSensingParams = targetSensingParams;
		this.targetInitialStates = targetInitialStates;

		this.myMotionHistory = trackerMotionHistory;
		this.mySensingParams = trackerSensingParams;
		this.myInitialState = trackerInitialState;

		this.obstacles = obstacles;
		this.goalRegion = goalRegion;
		initialise();
	}

	/**
	 * Initialises the tracker's policy.
	 * 
	 * This handles any setup your agent requires for its policy before the game
	 * actually starts. If you don't require any setup, leave this method blank.
	 */
	public void initialise() {
		// TODO Write this method!

		TargetGrid grid = targetPolicy.getGrid();

		GridCell next = targetPolicy.getNextIndex(grid
				.getCell(targetInitialStates.get(0).getPosition()));

		GridCell current = grid.getCell(targetInitialStates.get(0)
				.getPosition());/*
								 * System.out.println(current);
								 * System.out.println(next);
								 * 
								 * System.out.println("Tracker: " +
								 * myInitialState.getPosition());
								 * System.out.println("Target: " +
								 * targetInitialStates.get(0).getPosition());
								 * 
								 * System.out.println("Distance to target: " +
								 * TrackerTools
								 * .getDistanceToTarget(myInitialState,
								 * targetInitialStates.get(0)));
								 * System.out.println("Tracker heading angle: "
								 * + myInitialState.getHeading());
								 * System.out.println("Target heading angle: " +
								 * Math
								 * .toDegrees(targetInitialStates.get(0).getHeading
								 * ()));
								 * 
								 * System.out.println("Angle to target: " +
								 * TrackerTools.getAngleToTarget(myInitialState,
								 * targetInitialStates.get(0)));
								 */
		currentTarget = grid.getCell(targetInitialStates.get(0).getPosition());
		targetState = targetInitialStates.get(0);

	}

	@Override
	/**
	 * This method is used by the game to ask your tracker for an action
	 * when it is the tracker's turn.
	 * 
	 * It also passes your tracker all of the information it is allowed
	 * to have about the changing of the state of the game. In particular,
	 * this includes the following:
	 * 
	 * @param turnNo the current turn. This is the turn number within the game;
	 * this will be 0 for your very first turn, then 2, then 4, etc. 
	 * This is always even because odd-numbered turns are taken by the targets.
	 * 
	 * @param previousResult the result of the previous attempted action. This
	 * contains four components:
	 * previousResult.getDesiredAction() - the action that was previously
	 * 		attempted by this tracker, or null if there is no such action.
	 *		In other words, this is the output from the last time this
	 * 		method was called, or null if this is the first time this method
	 * 		is being called.
	 * previousResult.getDivergedAction() - the action that resulted after random
	 * 		divergence was applied to your desired action, or null if there was 
	 * 		no such action.
	 * previousResult.getResultingState() - the state this resulted in, which is
	 * 		also the current state of your tracker.
	 * previousResult.getReward() - the reward obtained for the previous
	 * action.
	 * 
	 * @param scores the scores accrued by each individual player.
	 * Note that player #0 is the tracker, and players #1, #2, etc. are
	 * targets. 
	 * In order to win the game, your score will need to be higher than
	 * the total of all of the target scores.
	 * Normally numTargets = 1, so scores will only consist of two numbers.\
	 * 
	 * @param newPercepts any percepts obtained by your tracker since
	 * its last turn.
	 * For example, if it's currently turn #2, this will contain any percepts
	 * from turns #0 and #1.
	 * The percept consists of three components:
	 * percept.getAgentNo() - the agent that was seen.
	 * percept.getTurnNo() - the turn after which it was seen.
	 * percept.getAgentState() - the state the agent was seen in.
	 */
	public TrackerAction getAction(int turnNo, ActionResult previousResult,
			double[] scores, List<Percept> newPercepts) {
		AgentState myState = previousResult.getResultingState();

		// TODO Write this method!
		TargetGrid grid = targetPolicy.getGrid();

		// get expected action from policy

		if (newPercepts.size() != 0) {
			AgentState agentState = newPercepts.get(newPercepts.size() - 1)
					.getAgentState();
			// heading = TrackerTools.getAngleToTarget(myState, agentState);
			// System.out.println("Heading angle: " + heading);
			targetState = agentState;
		}

		AgentState currentTargetState = targetState;
		targetState = targetPolicy.getAction(targetState).getResultingState();

		TrackerTools.maxUtility(2, targetPolicy,
				currentTargetState, targetMotionHistory, myState,
				targetSensingParams, mySensingParams, obstacles);
		
		return TrackerTools.a;
	}

	public int getActionCode(double[] probability) {
		int action = -1;
		double sum = 0;
		Random r = new Random();
		double random = r.nextDouble();
		for (int i = 0; i < 9; i++) {

			if (probability[i] == 0)
				continue;
			if (random >= sum && random < sum + probability[i]) {
				action = i;
				break;

			}
			sum = sum + probability[i];
		}

		if (action == -1) {
			// System.out.println(probability[0]);
		}
		return action;
	}
}
