package frc.robot.util;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

/**
 * A number you can edit live from the dashboard while Constants.kTuningMode is true.
 * With tuning mode off it is just a constant (get() returns the default and never touches
 * NetworkTables), so a stray dashboard value can't change match behavior.
 * Same API as 2910's LoggedTunableNumber, minus AdvantageKit. Values live under "/Tuning".
 */
public class TunableNumber implements DoubleSupplier {
    private static final NetworkTable kTable = NetworkTableInstance.getDefault().getTable("Tuning");

    private final double defaultValue;
    private final DoubleEntry entry; // null when tuning mode is off
    // One "last seen" value per caller id, so two subsystems watching the same number
    // each get their own change notification.
    private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();

    public TunableNumber(String key, double defaultValue) {
        this.defaultValue = defaultValue;
        if (Constants.kTuningMode) {
            entry = kTable.getDoubleTopic(key).getEntry(defaultValue);
            entry.set(defaultValue); // publish so it appears on the dashboard immediately
        } else {
            entry = null;
        }
    }

    public double get() {
        return entry == null ? defaultValue : entry.get(defaultValue);
    }

    /**
     * True the first time it's called for this id and whenever the value changed since
     * the last call with the same id. Pass hashCode() of the caller as the id.
     */
    public boolean hasChanged(int id) {
        final double currentValue = get();
        final Double lastValue = lastHasChangedValues.get(id);
        if (lastValue == null || currentValue != lastValue) {
            lastHasChangedValues.put(id, currentValue);
            return true;
        }
        return false;
    }

    /**
     * Runs action with the current values (in the order given) if ANY of the numbers changed.
     * Call from periodic(): cheap when nothing changed, reapplies gains when something did.
     */
    public static void ifChanged(int id, Consumer<double[]> action, TunableNumber... tunableNumbers) {
        if (Arrays.stream(tunableNumbers).anyMatch(number -> number.hasChanged(id))) {
            action.accept(Arrays.stream(tunableNumbers).mapToDouble(TunableNumber::get).toArray());
        }
    }

    @Override
    public double getAsDouble() {
        return get();
    }
}
