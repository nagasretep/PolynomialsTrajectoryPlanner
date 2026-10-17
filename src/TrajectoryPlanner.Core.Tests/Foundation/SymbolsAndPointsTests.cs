using TrajectoryPlanner.Core.BoundaryConditions;
using TrajectoryPlanner.Core.Points;
using TrajectoryPlanner.Core.States;
using TrajectoryPlanner.Core.Symbols;
using Xunit;

namespace TrajectoryPlanner.Core.Tests.Foundation;

public class SymbolsAndPointsTests
{
    [Fact]
    public void DerivativeOrder_HasCorrectIntegerValues()
    {
        Assert.Equal(0, (int)DerivativeOrder.Position);
        Assert.Equal(1, (int)DerivativeOrder.Velocity);
        Assert.Equal(2, (int)DerivativeOrder.Acceleration);
        Assert.Equal(3, (int)DerivativeOrder.Jerk);
        Assert.Equal(4, (int)DerivativeOrder.Snap);
    }

    [Fact]
    public void SupportedDegrees_AreCorrect()
    {
        Assert.Equal(new[] { 3, 5, 7, 9 }, TrajectoryConstants.SupportedDegrees);
    }

    [Fact]
    public void Point1D_BasicOperations()
    {
        var a = new Point1D(3.0);
        var b = new Point1D(7.0);

        Assert.Equal(10.0, (a + b).X, 1e-12);
        Assert.Equal(-4.0, (a - b).X, 1e-12);
        Assert.Equal(9.0, (a * 3).X, 1e-12);
        Assert.Equal(4.0, a.DistanceTo(b), 1e-12);
    }

    [Fact]
    public void Point3D_BasicOperations()
    {
        var a = new Point3D(1, 2, 3);
        var b = new Point3D(4, 5, 6);

        Assert.Equal(new Point3D(5, 7, 9), a + b);
        Assert.Equal(32, a.Dot(b), 1e-12);
        Assert.Equal(Math.Sqrt(14), a.Length, 1e-12);
    }

    [Fact]
    public void State1D_DefaultZero()
    {
        var s = State1D.Zero;
        Assert.Equal(0, s.Position);
        Assert.Equal(0, s.Velocity);
        Assert.Equal(0, s.Acceleration);
        Assert.Equal(0, s.Jerk);
        Assert.Equal(0, s.Snap);
    }

    [Fact]
    public void BoundaryConditionValidator_RequiredConditions()
    {
        Assert.Equal(4, BoundaryConditionValidator.RequiredConditionCount(3));
        Assert.Equal(6, BoundaryConditionValidator.RequiredConditionCount(5));
        Assert.Equal(8, BoundaryConditionValidator.RequiredConditionCount(7));
        Assert.Equal(10, BoundaryConditionValidator.RequiredConditionCount(9));
    }

    [Fact]
    public void BoundaryConditionValidator_DerivativeLevels()
    {
        Assert.Equal(2, BoundaryConditionValidator.DerivativeLevelsForDegree(3));
        Assert.Equal(3, BoundaryConditionValidator.DerivativeLevelsForDegree(5));
        Assert.Equal(4, BoundaryConditionValidator.DerivativeLevelsForDegree(7));
        Assert.Equal(5, BoundaryConditionValidator.DerivativeLevelsForDegree(9));
    }
}
