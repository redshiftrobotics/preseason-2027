package first.robot.utils;

import java.util.function.Consumer;

public class CollectionHelper {
    public static <T> void forEach(T[] array, Consumer<T> action) {
        for (int i = 0; i < array.length; i++) {
            action.accept(array[i]);
        }
    }

    private CollectionHelper() {}
}