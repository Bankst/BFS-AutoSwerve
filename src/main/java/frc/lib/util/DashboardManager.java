package frc.lib.util;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2. command.SubsystemBase;

import java.util.Collections;
import java.util.Map;

public class DashboardManager {

	private DashboardManager() {
		throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
	}

	public static Map<String, Object> getRangePropertiesMap(double both) {
		return Collections.unmodifiableMap(Map.of("Min", -both, "Max", both));
	}

	public static Map<String, Object> getRangePropertiesMap(double min, double max) {
		return Collections.unmodifiableMap(Map.of("Min", min, "Max", max));
	}

	/**
	 * Adds a tab on Shuffleboard. All other addTab* methods will do this automatically.
	 * @param tabName
	 */
	public static void addTab(String tabName) {
		Shuffleboard.getTab(tabName);
	}

	/**
	 * Adds a tab on Shuffleboard, with the Subsystem widget.
	 * If the tab already exists, the widget will be added.
	 * @param subsystemBase Subsystem to add a tab for
	 */
	public static void addTab(SubsystemBase subsystemBase) {
		var tab = Shuffleboard.getTab(subsystemBase.getName());
		try {
			tab.add(subsystemBase);
		} catch (IllegalArgumentException ex) {
			// ignore
		}
	}

	public static GenericEntry addTabItem(SubsystemBase subsystemBase, String itemName, Object defaultValue, WidgetType widget) {
		return addTabItem(subsystemBase, itemName, defaultValue, widget, Map.of());
	}

	public static GenericEntry addTabItem(SubsystemBase subsystemBase, String itemName, Object defaultValue, WidgetType widget, Map<String, Object> properties) {
		return addTabItem(subsystemBase.getName(), itemName, defaultValue, widget, properties);
	}

	public static GenericEntry addTabItem(String tabName, String itemName, Object defaultValue, WidgetType widget) {
		return addTabItem(tabName, itemName, defaultValue, widget, Map.of());
	}

	public static GenericEntry addTabItem(String tabName, String itemName, Object defaultValue, WidgetType widget, Map<String, Object> properties) {
		return Shuffleboard.getTab(tabName).add(itemName, defaultValue).withWidget(widget).withProperties(properties).getEntry();
	}

	public static GenericEntry addTabItem(String tabName, String itemName, Object defaultValue, WidgetType widget, int width, int height, Map<String, Object> properties) {
		return Shuffleboard.getTab(tabName).add(itemName, defaultValue).withWidget(widget).withSize(width, height).withProperties(properties).getEntry();
	}

	public static GenericEntry addTabItem(SubsystemBase subsystemBase, String itemName, Object defaultValue) {
		return addTabItem(subsystemBase.getName(), itemName, defaultValue);
	}

	public static GenericEntry addTabItem(String tabName, String itemName, Object defaultValue) {
		return addTabItem(tabName, itemName, defaultValue, BuiltInWidgets.kTextView);
	}

	
	public static GenericEntry addTabNumberBar(String tabName, String itemName, double min, double max) {
		return addTabItem(tabName, itemName, 0.0, BuiltInWidgets.kNumberBar, getRangePropertiesMap(min, max));
	}
	
	public static GenericEntry addTabNumberBar(SubsystemBase subsystemBase, String itemName, double min, double max) {
		return addTabNumberBar(subsystemBase.getName(), itemName, min, max);
	}

	public static GenericEntry addTabBooleanBox(String tabName, String itemName) {
		return addTabItem(tabName, itemName, false, BuiltInWidgets.kBooleanBox);
	}

	public static GenericEntry addTabBooleanBox(SubsystemBase subsystemBase, String itemName) {
		return addTabBooleanBox(subsystemBase.getName(), itemName);
	}

	public static GenericEntry addTabBooleanToggle(SubsystemBase subsystemBase, String itemName) {
		return addTabBooleanToggle(subsystemBase.getName(), itemName);
	}

	private static GenericEntry addTabBooleanToggle(String tabName, String itemName) {
		return addTabItem(tabName, itemName, false, BuiltInWidgets.kToggleButton);
	}

	public static GenericEntry addTabDial(SubsystemBase subsystemBase, String itemName, double min, double max) {
		return addTabDial(subsystemBase.getName(), itemName, min, max);
	}

	public static GenericEntry addTabDial(String tabName, String itemName, double min, double max) {
		return addTabItem(tabName, itemName, 0.0, BuiltInWidgets.kDial, getRangePropertiesMap(min, max));
	}

	public static void addTabButton(String tabName, String buttonName, Runnable onPress) {
		InstantCommand command = new InstantCommand(onPress);
		command.setName(buttonName);
		Shuffleboard.getTab(tabName).add(command);
	}

	public static void addTabSendable(SubsystemBase subsystem, String itemName, Sendable sendable) {
		addTabSendable(subsystem.getName(), itemName, sendable);
	}

	public static void addTabSendable(String tabName, String itemName, Sendable sendable) {
		Shuffleboard.getTab(tabName).add(itemName, sendable);
	}

	public static <E extends Enum<E>> SendableChooser<E> addTabChooser(String tabName, String chooserName, E[] choosable, E defaultChosen) {
		SendableChooser<E> chooser = new SendableChooser<>();

		for (E _enum : choosable) {
			if (_enum == defaultChosen) {
				chooser.setDefaultOption(_enum.name(), _enum);
			} else {
				chooser.addOption(_enum.name(), _enum);
			}
		}

		Shuffleboard.getTab(tabName).add(chooserName, chooser);

		return chooser;
	}
}