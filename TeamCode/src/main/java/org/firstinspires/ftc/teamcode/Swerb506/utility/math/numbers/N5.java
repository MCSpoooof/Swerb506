package org.firstinspires.ftc.teamcode.Swerb506.utility.math.numbers;

import org.firstinspires.ftc.teamcode.Swerb506.utility.math.Nat;
import org.firstinspires.ftc.teamcode.Swerb506.utility.math.Num;

/**
 * A class representing the number {{ num }}.
 */
public final class N5 extends Num implements Nat<N5> {



    private N5() {
    }


    /**
     * The integer this class represents.
     *
     * @return The literal number {{ num }}.
     */
    @Override
    public int getNum() {
        return 5;
    }

    /**
     * The singleton instance of this class.
     */
    public static final N5 instance = new N5();
}
