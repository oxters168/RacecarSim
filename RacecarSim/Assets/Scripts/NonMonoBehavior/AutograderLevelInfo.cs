﻿/// <summary>
/// Encapsulates information about an autograder level.
/// </summary>
public class AutograderLevelInfo
{
    /// <summary>
    /// A short word or phrase describing the objective of the level.
    /// </summary>
    public string Title;

    /// <summary>
    /// A longer description of the objective of the level.
    /// </summary>
    public string Description;

    /// <summary>
    /// The maximum number of points which the user can receive on this level.
    /// </summary>
    public float MaxPoints;

    /// <summary>
    /// The total time in seconds which the user can spend on this level.
    /// </summary>
    public float TimeLimit = 10;
}
