package com.team1816.lib;

//import com.google.inject.AbstractModule;
//import com.google.inject.Guice;
//import com.google.inject.Module;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * A wrapper for the GUICE injector to register modules and straighten instantiation pathways for run-time optimization
 */
public class Injector {
    private static final HashMap<Class<?>, Object> instances = new HashMap<>();
    private static final HashMap<Class<?>, Class<?>> classBindings = new HashMap<>();

    private static Inject _injector;

    private static List<Module> _modules = new ArrayList<>();

    /**
     * This method will initial the injector using the items in the
     * lib module and the passed in season module
     *
     * @param module This is the season module to register
     */
    public static void registerModule(Module module) {
        _modules.add(module);
    }

    /**
     * Registers an instance as a module
     *
     * @param instance
     * @param <T>
     */
    public static <T> void register(T instance) {
        instances.put(instance.getClass(), instance);
    }

    /**
     * Registers a class as a module
     *
     * @param type
     * @param instance
     * @param <T>
     */
    public static <T> void register(Class<T> type, Class<? extends T> instance) {
        classBindings.put(type, instance);
    }

    /**
     * Returns a module based on its associated class
     *
     * @param type
     * @param <T>
     * @return
     */
    public static <T> T get(Class<T> type) {
        // on first retrieval lock in the modules and create injector
        if (instances.containsKey(type)) {
            return type.cast(instances.get(type));
        }

        Class<?> implementation = classBindings.getOrDefault(type, type);
        try {
            T instance = type.cast(implementation.getDeclaredConstructor().newInstance());
            register(instance);
            return instance;
        } catch (Exception e) {
            throw new RuntimeException("Failed to instantiate class: " + implementation.getName(), e);
        }

    }
}
