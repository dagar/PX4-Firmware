@{
import genmsg.msgs

from px_generate_uorb_topic_helper import * # this is in Tools/

topic_name = spec.short_name

sorted_fields = sorted(spec.parsed_fields(), key=sizeof_field_type, reverse=True)
struct_size, padding_end_size = add_padding_bytes(sorted_fields, search_path)
topic_fields = ["%s %s" % (convert_type(field.type), field.name) for field in sorted_fields]
}@
@( ";".join(topic_fields) )
