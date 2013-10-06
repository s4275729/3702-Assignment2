package tracker;

import game.AgentState;

import java.util.ArrayList;
import java.util.List;

public class MDPState {
	private AgentState targetState;
	private AgentState trackerState;
	private double value = 0;
	private boolean isChanged = true;
	private int parentActionCode = -1;
	private double reward = 0;
	List<MDPState> children;
	
	public MDPState(AgentState targetState, AgentState trackerState) {
		this.setTargetState(targetState);
		this.setTrackerState(trackerState);
		children = new ArrayList<MDPState>();
	}


	/**
	 * @return the value
	 */
	public double getValue() {
		return value;
	}

	/**
	 * @param value the value to set
	 */
	public void setValue(double value) {
		this.value = value;
	}


	/**
	 * @return the targetState
	 */
	public AgentState getTargetState() {
		return targetState;
	}


	/**
	 * @param targetState the targetState to set
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
	 * @param trackerState the trackerState to set
	 */
	public void setTrackerState(AgentState trackerState) {
		this.trackerState = trackerState;
	}
	
	public void addChild(MDPState child) {
		children.add(child);
	}
	
	public boolean childExists(MDPState child) {
		for (int i = 0; i < children.size(); i++ ) {
			if (children.get(i).getTargetState() == child.getTargetState() &&
					children.get(i).getTrackerState() == child.getTrackerState()) {
				return true;
			}
		}
		return false;
	}

	public MDPState getChild(MDPState child) {
		for (int i = 0; i < children.size(); i++ ) {
			if (children.get(i).getTargetState() == child.getTargetState() &&
					children.get(i).getTrackerState() == child.getTrackerState()) {
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
	 * @param parentActionCode the parentActionCode to set
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
	 * @param isChanged the isChanged to set
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
	 * @param reward the reward to set
	 */
	public void setReward(double reward) {
		this.reward = reward;
	}

	
}
