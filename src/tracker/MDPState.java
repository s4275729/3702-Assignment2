package tracker;

import game.AgentState;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import divergence.MotionHistory;

public class MDPState {
	private AgentState targetState;
	private AgentState trackerState;
	private boolean isChanged = true;
	private int parentActionCode = -1;
	private double probability = 1;
	private double reward = 0;
	List<MDPState> children;
	HashMap<Integer, Double> rewardActions;
	HashMap<Integer, Double> valueActions;

	private int visited = 0;
	HashMap<Integer, Integer> actionsPerformed;

	public MDPState(AgentState targetState, AgentState trackerState) {
		this.setTargetState(targetState);
		this.setTrackerState(trackerState);
		children = new ArrayList<MDPState>();
		rewardActions = new HashMap<Integer, Double>();
		valueActions = new HashMap<Integer, Double>();
		actionsPerformed = new HashMap<Integer, Integer>();
	}

	public String toString() {
		String tostring = "tracker: " + this.trackerState + " target: " + this.targetState + "\n";
		
		for (int i = 0; i < children.size(); i ++) {
			tostring+="child" + children.get(i).toString();
		}
		return tostring;
		
	}
	public void setRewardAction(int action, double value) {
		rewardActions.put(action, value);
	}

	public void setValueAction(int action) {
		double value = 0;
		value += rewardActions.get(action);

		// += children * probability
		for (int i = 0; i < children.size(); i++) {
			if (children.get(i).getParentActionCode() == action) {
				double childValue =  children.get(i).getProbability()
						* children.get(i).getValue();
				value += childValue;
			}
		}
		valueActions.put(action, value);
	}

	public double getValue() {
		// return max of actions keys
		Double value = Double.MIN_VALUE;
		for (Entry<Integer, Double> entry : valueActions.entrySet()) {
			double comparisonValue = entry.getValue()
					+ Math.sqrt(((2 * Math.log(visited)) / actionsPerformed.get(entry
							.getKey())));
			if (comparisonValue > value) {
				value = entry.getValue();
			}
			//System.out.print(entry.getValue());
		}
		return value;
	}

	public int getAction() {
		// get action that returns highest value
		// return max of actions keys
		Double value = Double.MIN_VALUE;
		int actionKey = 0;
		for (Entry<Integer, Double> entry : valueActions.entrySet()) {
			double comparisonValue = entry.getValue()
					+ Math.sqrt(((2 * Math.log(visited)) / actionsPerformed.get(entry
							.getKey())));
			
			if (comparisonValue > value) {
				value = entry.getValue();
				actionKey = entry.getKey();
			}
		}
		return actionKey;
	}

	/**
	 * @return the targetState
	 */
	public AgentState getTargetState() {
		return targetState;
	}

	/**
	 * @param targetState
	 *            the targetState to set
	 */
	public void setTargetState(AgentState targetState) {
		this.targetState = targetState;
	}

	/**
	 * @return the trackerState
	 */
	public AgentState getTrackerState() {
		return trackerState;
	}

	/**
	 * @param trackerState
	 *            the trackerState to set
	 */
	public void setTrackerState(AgentState trackerState) {
		this.trackerState = trackerState;
	}

	public void addChild(MDPState child) {
		children.add(child);
	}

	public boolean childExists(MDPState child) {
		for (int i = 0; i < children.size(); i++) {
			if (children.get(i).getTargetState() == child.getTargetState()
					&& children.get(i).getTrackerState() == child
							.getTrackerState()
					&& children.get(i).getParentActionCode() == child
							.getParentActionCode()) {
				return true;
			}
		}
		return false;
	}

	public MDPState getChild(MDPState child) {
		for (int i = 0; i < children.size(); i++) {
			if (children.get(i).getTargetState() == child.getTargetState()
					&& children.get(i).getTrackerState() == child
							.getTrackerState()) {
				return children.get(i);
			}
		}
		return null;
	}

	/**
	 * @return the parentActionCode
	 */
	public int getParentActionCode() {
		return parentActionCode;
	}

	/**
	 * @param parentActionCode
	 *            the parentActionCode to set
	 */
	public void setParentActionCode(int parentActionCode) {
		this.parentActionCode = parentActionCode;
	}

	/**
	 * @return the isChanged
	 */
	public boolean isChanged() {
		return isChanged;
	}

	/**
	 * @param isChanged
	 *            the isChanged to set
	 */
	public void setChanged(boolean isChanged) {
		this.isChanged = isChanged;
	}

	/**
	 * @return the reward
	 */
	public double getReward() {
		return reward;
	}

	/**
	 * @param reward
	 *            the reward to set
	 */
	public void setReward(double reward) {
		this.reward = reward;
	}

	/**
	 * @return the probability
	 */
	public double getProbability() {
		return probability;
	}

	/**
	 * @param probability
	 *            the probability to set
	 */
	public void setProbability(double probability) {
		this.probability = probability;
	}

	/**
	 * @return the visited
	 */
	public int getVisited() {
		return visited;
	}

	/**
	 * @param visited
	 *            the visited to set
	 */
	public void setVisited(int visited) {
		this.visited = visited;
	}

}
