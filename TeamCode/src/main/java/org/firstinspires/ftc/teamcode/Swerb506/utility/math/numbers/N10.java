package org.firstinspires.ftc.teamcode.Swerb506.utility.math.numbers;

import org.firstinspires.ftc.teamcode.Swerb506.utility.math.Nat;
import org.firstinspires.ftc.teamcode.Swerb506.utility.math.Num;

/**
 * A class representing the number {{ num }}.
 */
public final class N10 extends Num implements Nat<N10> {



    private N10() {
    }


    /**
     * The integer this class represents.
     *
     * @return The literal number {{ num }}.
     */
    @Override
    public int getNum() {
        return 10;
    }

    /**
     * The singleton instance of this class.
     */
    public static final N10 instance = new N10();
}
