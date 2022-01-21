package frc.robot.auto;

public abstract class AutoRoutine {
    public void generate() {
	}

	public abstract void run();

	public String getName() {
		return this.getClass().getSimpleName();
	}

}
