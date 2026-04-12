package com.team1816.lib.subsystems.vision.filters;

import com.team1816.lib.subsystems.vision.results.ResultInterface;

public interface FilterInterface {

    boolean test(ResultInterface result);

    default FilterInterface and (FilterInterface other) {
        return (result) -> test(result) && other.test(result);
    }

    default FilterInterface or (FilterInterface other) {
        return (result) -> test(result) || other.test(result);
    }

    default FilterInterface negate() {
        return (result) -> !test(result);
    }

}
