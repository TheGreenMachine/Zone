package com.team1816.lib;

//import com.google.inject.AbstractModule;
//import com.google.inject.Guice;
//import com.google.inject.Module;

import java.lang.reflect.Constructor;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * A wrapper for the GUICE injector to register modules and straighten instantiation pathways for run-time optimization
 */
public class Injector {
    private static final HashMap<Class<?>, Object> instances = new HashMap<>();

    private static List<Module> _modules = new ArrayList<>();

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
        System.out.println(type);

        try {
            Constructor<?> milbert = null;
            Constructor<?>[] constructors = type.getConstructors();
            if(constructors.length == 1) { milbert = constructors[0]; }
            else {
                for (Constructor<?> cntr : constructors) {
                    if (cntr.isAnnotationPresent(Inject.class)) {
                        milbert = cntr;
                        break;
                    }
                }
            }
            if(milbert == null) throw new RuntimeException("No Annotation Present "+ type.getName());
//            Constructor<T> constructor = type.getDeclaredConstructor();
//            boolean tf = constructor.isAnnotationPresent(Inject.class);
//            if (!tf) throw new RuntimeException("No Annotation Present "+ type.getName());
            Class<?>[] pTypes = milbert.getParameterTypes();
            Object[] arg = new Object[pTypes.length];
            for(int i = 0; i < pTypes.length; i++){ arg[i] = get(pTypes[i]); }
            T instance = type.cast(milbert.newInstance(arg));
            register(instance);
            return instance;
        } catch (Exception e) {
            throw new RuntimeException("Failed to instantiate class: " + type.getName(), e);
        }

    }
}
