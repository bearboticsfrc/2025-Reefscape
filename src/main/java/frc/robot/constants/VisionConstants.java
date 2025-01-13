package frc.robot.constants;

public final class VisionConstants {

    public enum TAG {
        BLUE_FRONT_REEF(18),
        BLUE_R1_REEF(17),
        BLUE_R2_REEF(22),
        BLUE_L1_REEF(19),
        BLUE_L2_REEF(20),
        BLUE_BACK_REEF(21),

        RED_FRONT_REEF(7),
        RED_R1_REEF(8),
        RED_R2_REEF(9),
        RED_L1_REEF(6),
        RED_L2_REEF(11),
        RED_BACK_REEF(10),

        BLUE_BARGE_FRONT(14),
        BLUE_BARGE_BACK(4),
        
        RED_BARGE_FRONT(5),
        RED_BARGE_BACK(15),

        BLUE_PROC(3),
        RED_PROC(16),

        BLUE_COR_STAT_L(13),
        BLUE_COR_STAT_R(12),

        RED_COR_STAT_L(1),
        RED_COR_STAT_R(2),
        ;
    
        public final int tagNumber;
    
        public int getValue() {
          return tagNumber;
        }
    
        private TAG(int tagNumber) {
          this.tagNumber = tagNumber;
        }
      }
    
}
