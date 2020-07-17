@{
msg_names = [mn.replace(".msg", "") for mn in msgs]
msgs_count = len(msg_names)
msg_names_all = list(set(msg_names + multi_topics)) # set() filters duplicates
msg_names_all.sort()
msgs_count_all = len(msg_names_all)
}@
