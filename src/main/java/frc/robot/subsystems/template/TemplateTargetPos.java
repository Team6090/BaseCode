package frc.robot.subsystems.template;

public class TemplateTargetPos {

  private static TemplateTargetPos m_TargetPos = new TemplateTargetPos();

  public static TemplateTargetPos getInstance() {
    return m_TargetPos;
  }

  private static TargetedPos targetPos = TargetedPos.NONE;

  public static enum TargetedPos {
    NONE(0),
    ONE(1),
    TWO(2),
    THREE(3);

    private int id;

    TargetedPos(int id) {
      this.id = id;
    }

    public int getId() {
      return id;
    }
  }

  public static TargetedPos getTargetPos() {
    return targetPos;
  }

  public static void setTemplateTargetPos(TargetedPos targetPos) {
    TemplateTargetPos.targetPos = targetPos;
  }

  public static double getTargetTemplatePos() {
    switch (targetPos.id) {
      case 0: /* None */
        return TemplateConstants.zeroPos;
      case 1: /* One */
        return TemplateConstants.onePos;
      case 2: /* Two */
        return TemplateConstants.twoPos;
      case 3: /* Three */
        return TemplateConstants.threePos;
      default: /* Default to None */
        return TemplateConstants.zeroPos;
    }
  }
}
