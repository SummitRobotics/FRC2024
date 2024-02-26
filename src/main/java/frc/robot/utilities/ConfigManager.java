package frc.robot.utilities;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import edu.wpi.first.networktables.*;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

// Class to manage the robot's configuration parameters
public class ConfigManager {
    // private static final String CONFIG_FILE = "/home/lvuser/robot-config.json";
    private static final String CONFIG_FILE = "robot-config.json";
    private static final String TABLE_NAME = "TweakableParameters";
    private static final String ENTRY_NAME_PREFIX = String.format("/%s/", TABLE_NAME);

    // Constants to enable/disable the use of the config file and network table
    private static final Boolean USE_CONFIG_FILE = true;
    private static final Boolean USE_NETWORK_TABLE = true;

    private static ConfigManager instance; // Single instance
    // private final Gson gson = new Gson();
    private final Gson gson = new GsonBuilder().setPrettyPrinting().create();
    private RobotConfig config;
    private final NetworkTable table;
    private final Map<String, List<BiConsumer<String, Object>>> keyToCallbacks = new HashMap<>();
    private final Map<BiConsumer<String, Object>, List<String>> callbackToKeys = new HashMap<>();

    private ConfigManager() {
        if (USE_NETWORK_TABLE) {
            NetworkTableInstance inst = NetworkTableInstance.getDefault();
            table = inst.getTable(TABLE_NAME);
        } else { 
            table = null;
        }
        if (USE_CONFIG_FILE) {
            loadConfig();
        } else {
            config = new RobotConfig();
        }
    }

    // Public method to get the single instance of the class
    public static synchronized ConfigManager getInstance() {
        if (instance == null) {
            instance = new ConfigManager();
        }
        return instance;
    }

    // Load the config from the file
    public void loadConfig() {
        try (FileReader reader = new FileReader(CONFIG_FILE)) {
            config = gson.fromJson(reader, RobotConfig.class);
            if (config == null)
                config = new RobotConfig();
        } catch (IOException e) {
            e.printStackTrace();
            config = new RobotConfig();
        }
        if (USE_NETWORK_TABLE) {
            initializeNetworkTableEntries();
        }
    }

    // Save the current config to the file
    public void saveConfig() {
        try (FileWriter writer = new FileWriter(CONFIG_FILE)) {
            gson.toJson(config, writer);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // Initialize network table entries for each parameter in the config
    private void initializeNetworkTableEntries() {
        config.parameters.forEach((key, value) -> {
            NetworkTableEntry entry = table.getEntry(key);
            updateEntryWithValue(entry, value);
        });
    }

    // Update the value of an entry in the network table and save it to the config
    private void updateEntryWithValue(NetworkTableEntry entry, Object value) {
        // Set value in network table
        if (value instanceof Double) {
            entry.setDouble((Double) value);
        } else if (value instanceof String) {
            entry.setString((String) value);
        } else if (value instanceof Boolean) {
            entry.setBoolean((Boolean) value);
        }
        // Extend with other types as needed

        // Listen for changes in the network table and update the config
        NetworkTableInstance instance = table.getInstance();
        instance.addListener(entry, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event -> {
            if (entry.getName().startsWith(ENTRY_NAME_PREFIX)) {
                // Update the config with the new value
                String key = entry.getName().substring(ENTRY_NAME_PREFIX.length());
                Object newValue = event.valueData.value.getValue();
                config.parameters.put(key, newValue);
                
                // Save the config to the file
                if (USE_CONFIG_FILE) {
                    saveConfig();
                }

                // Trigger all callbacks associated with this key
                List<BiConsumer<String, Object>> callbacksForKey = keyToCallbacks.getOrDefault(key, Collections.emptyList());
                callbacksForKey.forEach(callback -> callback.accept(key, newValue));
            }
        });
    }

    // Register a callback to be triggered when the key is updated
    public void registerCallback(String key, BiConsumer<String, Object> callback) {
        registerCallback(Collections.singletonList(key), callback);
    }

    // Register a callback to be triggered when any of the keys in the list are
    // updated
    public void registerCallback(List<String> keys, BiConsumer<String, Object> callback) {
        callbackToKeys.put(callback, keys);
        keys.forEach(key -> {
            keyToCallbacks.computeIfAbsent(key, k -> new ArrayList<>()).add(callback);
            // Trigger the callback immediately with the current value for each key, if it
            // exists
            if (config.parameters.containsKey(key)) {
                callback.accept(key, config.parameters.get(key));
            }
        });
    }

    // Utility method with type supplier
    public <T> T get(String key, Supplier<T> defaultValueSupplier, Class<T> typeClass) {
        // Add to network table if it doesn't exist
        if (!config.parameters.containsKey(key)) {
            T defaultValue = defaultValueSupplier.get();
            config.parameters.put(key, defaultValue);

            // Save the config to the file
            if (USE_CONFIG_FILE) {
                saveConfig();
            }

            // Add to network table
            if (USE_NETWORK_TABLE) {
                NetworkTableEntry entry = table.getEntry(key);
                updateEntryWithValue(entry, defaultValue);
            }
            return defaultValue;
        }

        // Get value from config
        Object value = config.parameters.get(key);
        if (value != null && typeClass.isInstance(value)) {
            return typeClass.cast(value);
        }
        return defaultValueSupplier.get();
    }

    // Overloaded utility methods using suppliers for default values
    public double getDouble(String key, Supplier<Double> defaultValueSupplier) {
        return get(key, defaultValueSupplier, Double.class);
    }

    public String getString(String key, Supplier<String> defaultValueSupplier) {
        return get(key, defaultValueSupplier, String.class);
    }

    public boolean getBoolean(String key, Supplier<Boolean> defaultValueSupplier) {
        return get(key, defaultValueSupplier, Boolean.class);
    }

    // Overloaded method for direct default values
    public double getDouble(String key, double defaultValue) {
        return getDouble(key, () -> defaultValue);
    }

    public String getString(String key, String defaultValue) {
        return getString(key, () -> defaultValue);
    }

    public boolean getBoolean(String key, boolean defaultValue) {
        return getBoolean(key, () -> defaultValue);
    }

    // Overloaded methods which return suppliers
    public Supplier<Double> getDoubleSupplier(String key, double defaultValue) {
        return () -> getDouble(key, defaultValue);
    }

    public Supplier<String> getStringSupplier(String key, String defaultValue) {
        return () -> getString(key, defaultValue);
    }

    public Supplier<Boolean> getBooleanSupplier(String key, boolean defaultValue) {
        return () -> getBoolean(key, defaultValue);
    }

    // Nested class for prefixed configuration access
    public class PrefixedConfigAccessor {
        private final String prefix;

        public PrefixedConfigAccessor(String prefix) {
            this.prefix = prefix;
        }

        // Overloaded utility methods using suppliers for default values
        public double getDouble(String key, Supplier<Double> defaultValueSupplier) {
            return ConfigManager.this.get(prefix + key, defaultValueSupplier, Double.class);
        }

        public String getString(String key, Supplier<String> defaultValueSupplier) {
            return ConfigManager.this.get(prefix + key, defaultValueSupplier, String.class);
        }

        public boolean getBoolean(String key, Supplier<Boolean> defaultValueSupplier) {
            return ConfigManager.this.get(prefix + key, defaultValueSupplier, Boolean.class);
        }
        
        // Overloaded method for direct default values
        public double getDouble(String key, double defaultValue) {
            return ConfigManager.this.getDouble(prefix + key, defaultValue);
        }

        public String getString(String key, String defaultValue) {
            return ConfigManager.this.getString(prefix + key, defaultValue);
        }

        public boolean getBoolean(String key, boolean defaultValue) {
            return ConfigManager.this.getBoolean(prefix + key, defaultValue);
        }

        // Overloaded methods which return suppliers
        public Supplier<Double> getDoubleSupplier(String key, double defaultValue) {
            return ConfigManager.this.getDoubleSupplier(prefix + key, defaultValue);
        }

        public Supplier<String> getStringSupplier(String key, String defaultValue) {
            return ConfigManager.this.getStringSupplier(prefix + key, defaultValue);
        }

        public Supplier<Boolean> getBooleanSupplier(String key, boolean defaultValue) {
            return ConfigManager.this.getBooleanSupplier(prefix + key, defaultValue);
        }

        // Register callback with prefixed key
        public void registerCallback(String key, BiConsumer<String, Object> callback) {
            ConfigManager.this.registerCallback(prefix + key, callback);
        }

        // Register callback with prefixed keys, stripping the prefix before calling the callback
        public void registerCallback(List<String> keys, BiConsumer<String, Object> callback) {
            List<String> prefixedKeys = new ArrayList<>();
            keys.forEach(key -> prefixedKeys.add(prefix + key));
            ConfigManager.this.registerCallback(prefixedKeys, (name, neValue) -> {
                if (name.startsWith(prefix)) {
                    callback.accept(name.substring(prefix.length()), neValue);
                }
            });
        }
    }

    // Method to get a PrefixedConfigAccessor
    public PrefixedConfigAccessor getPrefixedAccessor(String prefix) {
        return new PrefixedConfigAccessor(prefix);
    }
}
