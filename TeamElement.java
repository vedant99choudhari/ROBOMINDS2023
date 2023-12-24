package org.firstinspires.ftc.teamcode;

public enum TeamElement {
    CENTER (50,35,70,55),
    RIGHT(120,45,140,65),
    LEFT(0,0,0,0);

    public int topX;
    public int topY;

    public int bottomX;
    public int bottomY;

    TeamElement(int topX, int topY, int bottomX, int bottomY) {
        this.topX = topX;
        this.topY = topY;
        this.bottomX = bottomX;
        this.bottomY = bottomY;
    }
}
