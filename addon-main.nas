var unload = func(addon) {
}

var main = func(addon) {
    
    logprint(LOG_INFO, "FGHaptic addon initialized from path ", addon.basePath);
    io.load_nasal(addon.basePath ~ "/Nasal/force_feedback.nas");
    #setlistener("/sim/signals/nasal-dir-initialized", force_feedback.force_feedback_main_init);
    print("Haptic loaded");
}
